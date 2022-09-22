#include "message_interfaces/msg/gnss.hpp"
#include  "gnss/ntzx_qianxun_inv.h"
#include "gnss/ntzx_log_app.h"
#include "gnss/ntzx_conf_app.h"
#include  "gnss/gnss.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>


using namespace std;

#define QIANXUN_TITLE "NAV"
#define QIANXUN_USART_KEY "USART_NAV"

/* 定义惯导盒子的物理接口 */
#define COMPORT1 "COM1"
#define COMPORT2 "COM2"

#define QIANXUN_RECV_DATA_LEN 512

/* 有关惯导配置的指令和相关宏定义 */
#define UNLOG_TYPE 0 // 清楚所有打印输出
#define GPTRA_TYPE 1 // 接收惯导的姿态信息
#define SAVE_TYPE 2  // 保存配置
#define GPGGA_TYPE 3 // 接收惯导信息
#define GPHDT_TYPE 4 // 接收航向角信息

/* 标志位 */
#define QIANXUN_GPGGA_SYNC_FLAG 0x01
#define QIANXUN_GPHDT_SYNC_FLAG 0x02

/* 数据同步完成 */
#define QIANXUN_DATA_SYNC_FALG (QIANXUN_GPGGA_SYNC_FLAG | QIANXUN_GPHDT_SYNC_FLAG)

#define UNLOG_CMD "UNLOG"
#define GPTRA_CMD "GPTRA"
#define SAVE_CMD "SAVECONFIG"
#define GPGGA_CMD "GPGGA"
#define GPHDT_CMD "GPHDT"

#define RECV_GPTRA_HEAD "$GPTRA"
#define QIANXUN_GPGGA_HEAD "$GPGGA"
#define QIANXUN_GPHDT_HEAD "$GNHDT"

/* 定义数据分隔符 */
#define QIANXUN_SEPARATOR ','

static int g_ntzx_qianxun_ufd = -1;
static char g_qianxun_tty_dev_name[20] = {0};
static char g_qianxun_recv_data[QIANXUN_RECV_DATA_LEN] = {0};
static char g_proc_flag = 0;
static int g_gps_data_frame = 0;
static str_gps_data g_gps_data = {0};
static int g_qanxun_data_sync_flag = 0;
// ntzx_systimer clean_timer = {0};
/* 数据锁 */
static pthread_mutex_t g_mutex_qianxun_gps_info = PTHREAD_MUTEX_INITIALIZER;

static int ntzx_qianxun_usart_init(void);
static int ntzx_qianxun_send(char *recv_buf, int recv_len);
static int ntzx_qianxun_config_judge(void);
static int ntzx_qianxun_config(int cmd_type, float upload_v);
static int ntzx_gpgga_proc(void);
static int ntzx_gphdt_proc(void);
static int ntzx_gptra_proc(void);
static int ntzx_qianxun_recv(char *recv_buf, int recv_len);
static void *ntzx_qianxun_inv_recv(void *arg);
static void *ntzx_qianxun_inv_proc(void *arg);
static void ntzx_qianxun_restart(void);


#define QIANXUN_GPGGA_SYNC_FLAG 0x01

#define QIANXUN_GPGGA_HEAD "$GPYBM"
#define QIANXUN_RECV_DATA_LEN 1024
#define QIANXUN_SEPARATOR ','

static str_vel_data g_vel_data = {0};
FILE *fp=NULL;
string temp;


class TopicPublisher01 : public rclcpp::Node
{
public:
  TopicPublisher01(std::string name) : Node(name)
    {
        // RCLCPP_INFO(this->get_logger(),"我是%s,发布话题为:/command.",name.c_str());
        gnss_command_publisher_ = this->create_publisher<message_interfaces::msg::Gnss>("gnss_command",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&TopicPublisher01::timer_callback,this));
    }
private:
   void timer_callback()
    {
        message_interfaces::msg::Gnss message;
        /*时间数据发布*/
        message.hour = to_string(g_gps_data.time.hour);
        message.min = to_string(g_gps_data.time.min);
        message.s = to_string(g_gps_data.time.s);
        message.ms = to_string(g_gps_data.time.ms);
        RCLCPP_INFO(this->get_logger(),"time='%s':'%s':'%s':'%s'",message.hour.c_str(),message.min.c_str(),message.s.c_str(),message.ms.c_str());
        /*经度数据发布*/
        message.lon= to_string(g_gps_data.lon);
        RCLCPP_INFO(this->get_logger(),"lon='%s'",message.lon.c_str());
        /*纬度数据发布*/
        message.lat= to_string(g_gps_data.lat);
        RCLCPP_INFO(this->get_logger(),"lat='%s'",message.lat.c_str());
        /*航向角数据发布*/
        message.courseangle= to_string(g_gps_data.courseAngle);
        RCLCPP_INFO(this->get_logger(),"courseAngle='%s'",message.courseangle.c_str());
        /*俯仰角数据发布*/
        message.picth= to_string(g_gps_data.Picth);
        RCLCPP_INFO(this->get_logger(),"Picth='%s'",message.picth.c_str());
        /*北向速度数据发布*/
        message.vel_n= to_string(g_vel_data.Vel_N);
        RCLCPP_INFO(this->get_logger(),"vel_n='%s'",message.vel_n.c_str());
        gnss_command_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<message_interfaces::msg::Gnss>::SharedPtr gnss_command_publisher_;

};





/* GPYBM处理函数 */
static int ntzx_gpgga_proc(void)
{
    char *p_GPYBM = NULL;
    char *q_GPYBM = NULL;

    char dev_GPYBM[] = "SNxxxxxxxx";
    int time_GPYBM = 0;
    double lon = 0;
    double lat = 0;
    float ElpHeight_GPYBM = 0;
    double Heading_GPYBM = 0;
    double Pitch_GPYBM = 0;
    float Vel_N_GPYBM = 0;
    float Vel_E_GPYBM = 0;
    float Vel_D_GPYBM = 0;
    float Vel_G_GPYBM = 0;
    float Coordinnate_Northing_GPYBM = 0;
    float Coordinnate_Easting_GPYBM = 0;
    float North_Distance_GPYBM = 0;
    float East_Distance_GPYBM = 0;
    unsigned char Position_Indicator_GPYBM = 0;
    unsigned char Heading_Indicator_GPYBM = 0;
    unsigned char SVn_GPYBM = 0;
    float Diff_Age_GPYBM = 0;
    unsigned int Station_ID_GPYBM = 0;
    float Baseline_length_GPYBM = 0;
    unsigned int Solution_sv_GPYBM = 0;
    float rolling_GPYBM = 0;


    int i = 0,j = 0;
    for (; i < strlen(g_qianxun_recv_data); i++) {
        if (g_qianxun_recv_data[i] == QIANXUN_SEPARATOR) {
            j++;
        }
    }
    if (j != 23) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    p_GPYBM = strchr(g_qianxun_recv_data, '*');
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    p_GPYBM = strchr(g_qianxun_recv_data, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    // printf("%s\n",g_qianxun_recv_data);
    /*获取设备号*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM, QIANXUN_SEPARATOR);
    if(q_GPYBM == NULL){
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    strcpy(dev_GPYBM,p_GPYBM);
    printf("dev=%s\n",dev_GPYBM);

    /*获取北京时间,获取到的时间格式为hhmmss.ss,小数点后的ss为毫秒*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    time_GPYBM = (int)(atof(q_GPYBM) * 100);
    printf("time_GPYBM=%d\n",time_GPYBM);

    /*获取纬度信息*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    lon = atof(p_GPYBM);
    printf("lon=%.3f\n",lon);

    /*获取经度信息*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    lat = atof(q_GPYBM);
    printf("lat=%.3f\n",lat);

    /*获取椭球高*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    ElpHeight_GPYBM = (float)atof(p_GPYBM);
    printf("ElpHeight_GPYBM=%.3f\n",ElpHeight_GPYBM);

    /*获取航向角*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Heading_GPYBM = atof(q_GPYBM);
    printf("Heading_GPYBM=%.3f\n",Heading_GPYBM);

    /*获取俯仰角*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Pitch_GPYBM = atof(p_GPYBM);
    printf("Pitch_GPYBM=%.3f\n",Pitch_GPYBM);

    /*获取北方向速度*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Vel_N_GPYBM = (float)atof(q_GPYBM);
    printf("Vel_N_GPYBM=%.3f\n",Vel_N_GPYBM);

    /*获取东方向速度*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Vel_E_GPYBM = (float)atof(p_GPYBM);
    printf("Vel_E_GPYBM=%.3f\n",Vel_E_GPYBM);

    /*获取地向速度*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Vel_D_GPYBM = (float)atof(q_GPYBM);
    printf("Vel_D_GPYBM=%.3f\n",Vel_D_GPYBM);

    /*获取地面速度*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Vel_G_GPYBM = (float)atof(p_GPYBM);
    printf("Vel_G_GPYBM = %.3f\n",Vel_G_GPYBM);
    
    /*获取高斯投影坐标X轴*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Coordinnate_Northing_GPYBM = atof(q_GPYBM);
    printf("Coordinnate_Northing_GPYBM=%.3f\n",Coordinnate_Northing_GPYBM);

    /*获取高斯坐标Y轴*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Coordinnate_Easting_GPYBM = atof(p_GPYBM);
    printf("Coordinnate_Easting_GPYBM=%f\n",Coordinnate_Easting_GPYBM);
    //////////////////
    /*获取基站坐标系下的移动站X坐标*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);

    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    North_Distance_GPYBM = atof(q_GPYBM);
    printf("North_Distance_GPYBM=%.3f\n",North_Distance_GPYBM);
    /*获取基站坐标系下的移动站Y坐标*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    East_Distance_GPYBM = atof(p_GPYBM);
    printf("East_Distance_GPYBM=%.3f\n",East_Distance_GPYBM);

    /*定位解状态*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Position_Indicator_GPYBM = (unsigned char)atoi(q_GPYBM);
    printf("Position_Indicator_GPYBM=%d\n",Position_Indicator_GPYBM);

    /*定位解状态*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Heading_Indicator_GPYBM = (unsigned char)atoi(p_GPYBM);
    printf("Heading_Indicator_GPYBM=%d\n",Heading_Indicator_GPYBM);

    /*主站天线收星数*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    SVn_GPYBM = (unsigned char)atoi(q_GPYBM);
    printf("SVn_GPYBM=%d\n",SVn_GPYBM);

    /*差分延迟*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Diff_Age_GPYBM = (float)atof(p_GPYBM);
    printf("Diff_Age_GPYBM=%.3f\n",Diff_Age_GPYBM);

    /*基准站ID*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Station_ID_GPYBM = (unsigned short)atoi(q_GPYBM);
    printf("Station_ID_GPYBM=%d\n",Station_ID_GPYBM);

    /*主站到从站距离*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Baseline_length_GPYBM = (float)atof(p_GPYBM);
    printf("Baseline_length_GPYBM=%.3f\n",Baseline_length_GPYBM);

    /*从站参与解算的卫星数*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Solution_sv_GPYBM = (unsigned short)atoi(q_GPYBM);
    printf("Solution_sv_GPYBM=%d\n",Solution_sv_GPYBM);

    // /*校验值*/
    p_GPYBM++;
    
    q_GPYBM = strchr(p_GPYBM,'*');
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    // printf("\n");

    // rolling_GPYBM = (float)atof(p_GPYBM);
    // printf("rolling_GPYBM=%.3f\n",rolling_GPYBM);
    // /*校验值*/
    // q_GPYBM++;
    
    // p_GPYBM = strchr(q_GPYBM, '*');
    // if (p_GPYBM == NULL) {
    //     return NTZX_QIANXUN_ANALYSIS_ERR;
    // }
    // *p_GPYBM = 0;
    
    //////////

/*解析成功将数据写入结构体*/
    pthread_mutex_lock(&g_mutex_qianxun_gps_info);
    g_gps_data.time.hour = (unsigned char)(((time_GPYBM / 1000000) + 8) % 24); //+8是因为北京时间与世界时间差8个小时
    g_gps_data.time.min = (unsigned char)((time_GPYBM % 1000000) / 10000);
    g_gps_data.time.s = (unsigned char)((time_GPYBM % 10000) / 100);
    g_gps_data.time.ms = (unsigned short)(time_GPYBM % 100) * 10;
    g_gps_data.lat = lat;
    g_gps_data.lat_dir = 0;
    g_gps_data.lon = lon;
    g_gps_data.lon_dir = 0;
    g_gps_data.courseAngle = Heading_GPYBM;   //航向角
    g_gps_data.Picth = Pitch_GPYBM;           //附仰角
    
     fprintf(fp,"time=%d:%d:%d:%d,lat=%0.4f,lon=%0.4f,courseAngle=%0.4f,Pitch=%0.4f,vel_n=%0.4f\n", g_gps_data.time.hour ,g_gps_data.time.min, g_gps_data.time.s, g_gps_data.time.ms, g_gps_data.lat,g_gps_data.lon, g_gps_data.courseAngle,g_gps_data.Picth,g_vel_data.Vel_N);
    fflush(fp);


    g_vel_data.Vel_N = Vel_N_GPYBM;     //速度
    g_vel_data.Vel_E = Vel_E_GPYBM;
    g_vel_data.Vel_G = Vel_G_GPYBM;
    g_vel_data.Vel_D = Vel_D_GPYBM;
    
    
    g_qanxun_data_sync_flag |= QIANXUN_GPGGA_SYNC_FLAG;
    pthread_mutex_unlock(&g_mutex_qianxun_gps_info);

    // printf("gps.lat=%f,gps.lon=%f,couseangle=%f,vel=%f\n",g_gps_data.lat,g_gps_data.lon,g_gps_data.courseAngle,g_vel_data.Vel_N);

    return NTZX_QIANXUN_SUCCESS;
}
static int ntzx_qianxun_recv(char *recv_buf, int recv_len)
{
    int rt;
    int i;
    if (recv_buf == NULL || recv_len < 0)
    {
        // cout<<"1"<<endl;
        return NTZX_QIANXUN_PARA_ERR;
    }
    // cout<<"2"<<endl;
    for (i = 0; i < recv_len; i++)
    {
        rt = ntzx_usart_timeout_recv(g_ntzx_qianxun_ufd, &recv_buf[i], 1);
        if (rt <= 0)
        {
            // cout<<"3"<<endl;
            return NTZX_QIANXUN_RECV_ERR;
        }
        // cout<<"4"<<endl;
        if (recv_buf[i] == '\n')
        {
            // cout<<"5"<<endl;
            recv_buf[i + 1] = 0;
            return NTZX_QIANXUN_SUCCESS;
        }
        // cout<<"6"<<endl;
    }
    // return NTZX_QIANXUN_SUCCESS;
   return NTZX_QIANXUN_RECV_ERR;
}

static void *ntzx_qianxun_inv_proc(void *arg)
{

    // cout<<"go_in"<<endl;
    char *p = NULL;
    int rt;
    while (1)
    {
        if (g_proc_flag == 0)
        {
            usleep(1000);
            continue;
        }
        /* GPYBM导航信息 */
        p = strstr(g_qianxun_recv_data, QIANXUN_GPGGA_HEAD);
        if (p != NULL)
        { 
            rt = ntzx_gpgga_proc();
            // cout<<rt<<endl;
            g_proc_flag = 0;
            continue;
        }
        g_proc_flag = 0;
        qianxun_flush();
    }
}

static void *ntzx_qianxun_inv_recv(void *arg)
{
    struct_usart_init_info mc_usart_info;
    mc_usart_info.baudrate = 115200;
    mc_usart_info.databit = 8;
    mc_usart_info.fctl = 0;
    mc_usart_info.parity = 0;
    mc_usart_info.stopbit = 1;
    mc_usart_info.tty_dev_name = "/dev/ttyUSB1";
    // mc_usart_info.tty_dev_name = "/dev/pts/4";
    g_ntzx_qianxun_ufd = ntzx_usart_init(&mc_usart_info);
    if (g_ntzx_qianxun_ufd>= 0) {
        cout<<"usart init success"<<endl;
    }
    else{
      cout<<"usart init fail"<<endl;
      cout<<"waiting..."<<endl;
      };
    char recv_buf[QIANXUN_RECV_DATA_LEN] = {0};
    int timeout_time = 0;
    int init_flag = 0; // 由于惯导热启动和冷启动时间问题，所以第一次上数据时间不超时
    /* 接收惯导数据 */
    while (1)
    {
        if (g_ntzx_qianxun_ufd > 0)
        {
            if (ntzx_qianxun_recv(recv_buf, sizeof(recv_buf)) < 0)
            {
                cout<<"failed"<<endl;
                usleep(1000);
                /* 长时间没有收到数据则重启 */
                if (init_flag == 1)
                {
                    timeout_time++;
                }
                continue;
            }
             cout<<"success"<<endl;
            timeout_time = 0;
            init_flag = 1;
            while (g_proc_flag != 0)
            {
                usleep(100);
            }
            /* 最后一个参数+1是为将接收到的buf的最后以为\0字符也写入进去 */
            memcpy(g_qianxun_recv_data, recv_buf, strlen(recv_buf) + 1);
            cout<<"g_qianxun_recv_data"<<g_qianxun_recv_data<<endl;
            g_proc_flag = 1;
            usleep(100);
        }
    }
}

void qianxun_flush(void)
{
    tcflush(g_ntzx_qianxun_ufd, TCIFLUSH);
}


int main(int argc, char** argv) {

  /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    
    /*产生一个node_01的节点*/
    auto node = std::make_shared<TopicPublisher01>("gnss_publisher");
    // 打印一句自我介绍
    
    RCLCPP_INFO(node->get_logger(), "gnss_publisher节点已经启动.");
    // int rt;

    fp=fopen("gnss_data.txt","w");

    pthread_t thread_inv_proc;
    pthread_t thread_inv_recv;
    

  /* 写入千寻惯导初始化成功 */
    cout<<"qianxun init success!!!"<<endl;
  
  pthread_create(&thread_inv_recv, NULL, ntzx_qianxun_inv_recv, NULL);

    /* 创建惯导信息处理线程 */
  pthread_create(&thread_inv_proc, NULL, ntzx_qianxun_inv_proc, NULL);

    cout<<"recv and proc success!!!"<<endl;
    // pthread_join(thread_inv_recv, NULL);
    // pthread_join(thread_inv_proc, NULL);
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
    }

