#include "mc/minimal_publish.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pthread.h>
#include <unistd.h>
#include <cstdlib>
#include "mc/mc_sta.hpp"
#include "message_interfaces/msg/control.hpp"
#include "message_interfaces/msg/gnss.hpp"
#include "message_interfaces/msg/imu.hpp"
#include "message_interfaces/msg/pl.hpp"
#include "message_interfaces/msg/speedwrite.hpp"


struct_mc_sta mc_sta;
using namespace std;
static int g_usart_fd = -1;
int left_high = 0;
int left_low = 0;
int right_high = 0;
int right_low = 0;
int V_cx=0;
int W_z=0;
unsigned char g_mc_ctrl_string[] = { 237 ,222, 23 ,4, 2, 128, 6,80,1, 80, 1, 80, 1, 80, 1, 0, 0, 0, 0, 178, 3, 13, 10 };//mc底层格式


class TopicSuscribe01 : public rclcpp::Node
{
public:
    TopicSuscribe01(std::string name) : Node(name)
    {
        //RCLCPP_INFO(this->get_logger(),"我是%s，订阅话题为：/command.",name.c_str());
        control_subscriber_ = this->create_subscription<message_interfaces::msg::Speedwrite>("pl_command",10,std::bind(&TopicSuscribe01::control_command_callback,this,std::placeholders::_1));
        gnss_subscriber_ = this->create_subscription<message_interfaces::msg::Gnss>("gnss_command",10,std::bind(&TopicSuscribe01::gnss_command_callback,this,std::placeholders::_1));
        imu_subscriber_ = this->create_subscription<message_interfaces::msg::Imu>("imu_command",10,std::bind(&TopicSuscribe01::imu_command_callback,this,std::placeholders::_1));
        // pl_subscriber_ = this->create_subscription<message_interfaces::msg::Pl>("pl_command",10,std::bind(&TopicSuscribe01::pl_command_callback,this,std::placeholders::_1));
    }
private:
    rclcpp::Subscription<message_interfaces::msg::Speedwrite>::SharedPtr control_subscriber_;
    rclcpp::Subscription<message_interfaces::msg::Gnss>::SharedPtr gnss_subscriber_;
    rclcpp::Subscription<message_interfaces::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<message_interfaces::msg::Pl>::SharedPtr pl_subscriber_;

    void control_command_callback(const message_interfaces::msg::Speedwrite::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(),"left_high:%s\tleft_low:%s\tright_high:%s\tright_low:%s",msg->left_high.c_str(),msg->left_low.c_str(),msg->right_high.c_str(),msg->right_low.c_str());
      
      left_high = atoi(msg->left_high.c_str());
      left_low = atoi(msg->left_low.c_str());
      right_high = atoi(msg->right_high.c_str());
      right_low = atoi(msg->right_low.c_str());
      cout<<"left_high"<<left_high<<endl;
      cout<<"left_low"<<left_low<<endl;
      // if(abs(atoi(msg->v_c.c_str())) >= 1000)
      // {
      //   V_cx =1000;
      // }else{V_cx = atoi(msg->v_c.c_str());}
      
      // if(abs(atoi(msg->w_c.c_str())) >= 60)
      // {
      //   W_z = 60;
      // // }else{W_z = atoi(msg->w_c.c_str());}
      // V_cx = atoi(msg->v_c.c_str());
      // W_z = atoi(msg->w_c.c_str());
      // cout<<V_cx<<endl;
      // cout<<W_z<<endl;
    };

    void gnss_command_callback(const message_interfaces::msg::Gnss::SharedPtr msg)
    {   
        //时间 经度 纬度 航向角 俯仰角
        // RCLCPP_INFO(this->get_logger(), 
        // "\n/gnss_command收到的\n\
        // time = '%s':'%s':'%s':'%s'\n\
        // lat=[%s]\tlon=[%s]\ncourseangle=[%s]\tpicth=[%s]\n/gnss_command数据接收OK\n", 
        // msg->hour.c_str(), msg->min.c_str(), msg->s.c_str(), msg->ms.c_str(), 
        // msg->lat.c_str(), msg->lon.c_str(), 
        // msg->courseangle.c_str(), msg->picth.c_str());
    };
        
    void imu_command_callback(const message_interfaces::msg::Imu::SharedPtr msg)
    {   
        //加速度 角速度 欧拉角 磁场
        // RCLCPP_INFO(this->get_logger(), 
        // "\n/imu_command收到的\n\
        // acceleration_x=[%s]\tacceleration_y=[%s]\tacceleration_z=[%s]\n\
        // angular_velocity_x=[%s]\tangular_velocity_y=[%s]\tangular_velocity_z=[%s]\n\
        // angle_degree_x=[%s]\tangle_degree_y=[%s]\tangle_degree_z=[%s]\n\
        // // magnetometer_x=[%s]\tmagnetometer_y=[%s]\tmagnetometer_z=[%s]\n/imu_command数据接收OK\n", 
        // msg->acceleration_x.c_str(), msg->acceleration_y.c_str(), msg->acceleration_z.c_str(), 
        // msg->angular_velocity_x.c_str(), msg->angular_velocity_y.c_str(), msg->angular_velocity_z.c_str(), 
        // msg->angle_degree_x.c_str(), msg->angle_degree_y.c_str(), msg->angle_degree_z.c_str(), 
        // msg->magnetometer_x.c_str(), msg->magnetometer_y.c_str(), msg->magnetometer_z.c_str());
    };

    void pl_command_callback(const message_interfaces::msg::Pl::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), 
        "\n/pl_command收到的\n\
        speed_mm=[%s]\tsteer_angle=[%s]\n/imu_command数据接收OK\n", 
        msg->speed_mm.c_str(), msg->steer_angle.c_str());
        
        V_cx = atoi(msg->speed_mm.c_str());
        W_z = (int)(atoi(msg->steer_angle.c_str())/57.3);
        cout<<"!!!!!!"<<V_cx<<endl;
        cout<<"??????"<<W_z<<endl;
    };

};
int ntzx_recv_data_handle(unsigned char* row){
  int i = 0;
    for ( i = 0; i < 128; i++)
    {
        if (row[i] == 237 && row[i+1] == 222 ) //报文以237 、222为开头（ED DE）
        {   
            if (i > 39){ //小车上传报文总长为25，若检查到39位也没找到开头，则判定接受数据失败
                return NTZX_MC_RECV_INFO_ERR;		
            }
            if (row[i + 23] == 13 && row[i + 24] == 10)   //报文以13 、10为结尾（0D 0A）
            {
              //0-3对应四个轮子的速度
              int Right_wheel_speed = row[i + 8]  * 256 + row[i + 7];
              //判断是否为负数
              if (row[i + 8]>=128) {
                  Right_wheel_speed= Right_wheel_speed-65536;
              }
              int Left_wheel_speed = row[i + 10] * 256 + row[i + 9];
                if (row[i + 10] >= 128) {
                  Left_wheel_speed = Left_wheel_speed - 65536;
              }

              if (Right_wheel_speed == 0 && Left_wheel_speed == 0)
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_STOP;
                    mc_sta.speed_mm=0;
                    mc_sta.steer_angle = 0;
                }
              else if (abs(Right_wheel_speed-Left_wheel_speed)<=10 && Right_wheel_speed >0)
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_GO_FORWARD;
                    mc_sta.speed_mm = Right_wheel_speed;
                    mc_sta.steer_angle = 0;
                }
                else if (abs(Right_wheel_speed-Left_wheel_speed)<=10 && Right_wheel_speed <0)
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_GO_BACKED;
                    mc_sta.speed_mm = Right_wheel_speed;
                    mc_sta.steer_angle = 0;
                }
                else if ((Right_wheel_speed-Left_wheel_speed)>10 && Left_wheel_speed>=0)
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_TURN_LEFT;
                    mc_sta.speed_mm = Right_wheel_speed;
                    float rad = ((float) (Right_wheel_speed-Left_wheel_speed))/CAR_LEN;
                    int  turn_degree = (int)(rad*RAD2DEGREE);
                    mc_sta.steer_angle = turn_degree;
                }
                else if ((Left_wheel_speed - Right_wheel_speed)>10 &&Right_wheel_speed>=0)
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_TURN_RIGHT;
                    mc_sta.speed_mm  = Left_wheel_speed;
                    float rad = ((float) (Left_wheel_speed - Right_wheel_speed))/CAR_LEN;
                    int  turn_degree = (int)(rad*RAD2DEGREE);
                    mc_sta.steer_angle = turn_degree; 
                }
                else if ( Left_wheel_speed < 0 && Right_wheel_speed >0  )
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_TWIRL_RIGHT;
                    mc_sta.speed_mm = Right_wheel_speed;
                }
                else if( Left_wheel_speed >0 && Right_wheel_speed <0)
                {
                    mc_sta.action_sta = NTZX_MC_ACTION_TWIRL_LEFT;
                    mc_sta.speed_mm = Left_wheel_speed;
                }
                // //4为电压，5为小车的状态
                mc_sta.vol = (row[i + 16] * 256 + row[i + 15]);
                mc_sta.chassis_sta= row[i + 20] * 256 + row[i + 19];
                cout<<"action_sta = "<<mc_sta.action_sta<<endl;
                cout<<"speed_mm = "<<mc_sta.speed_mm<<endl;
                cout<<"steer_angle = "<<mc_sta.steer_angle<<endl;
                cout<<"io_sta = "<<mc_sta.io_sta<<endl;
                cout<<"chassis_sta = "<<mc_sta.chassis_sta<<endl;
                cout<<"vol = "<<mc_sta.vol<<endl;
                return NTZX_MC_SUCCESS;
          }
           
        }     
        
    }
    // 未找到报文开发
    return NTZX_MC_RECV_INFO_ERR;
}

void* thread_uasrtsend(void* args){
    struct_usart_init_info mc_usart_info;
    mc_usart_info.baudrate = 115200;
    mc_usart_info.databit = 8;
    mc_usart_info.fctl = 0;
    mc_usart_info.parity = 0;
    mc_usart_info.stopbit = 1;
    // mc_usart_info.tty_dev_name = "/dev/ttyS1";
    // mc_usart_info.tty_dev_name = "/dev/pts/5";
    mc_usart_info.tty_dev_name = "/dev/ttyUSB0";
    g_usart_fd = ntzx_usart_init(&mc_usart_info);
    if (g_usart_fd >= 0) {
        cout<<"usart init success"<<endl;
    }
    else{
      cout<<"usart init fail"<<endl;
      cout<<"waiting..."<<endl;
      sleep(2);
      thread_uasrtsend(args);
      };
    while(1){
    //初始化地盘数据
      unsigned char g_mc_ctrl_string[] = { 237 ,222, 23 ,4, 2, 128, 6,80,1, 80, 1, 80, 1, 80, 1, 0, 0, 0, 0, 178, 3, 13, 10 };//mc底层格式
      // unsigned char V_l_low,V_l_high,V_r_low,V_r_high;
      // //逆运动学
      // int V_l,V_r;
      // V_l = V_cx-W_z*0.5*620;
      // V_r = V_cx+W_z*0.5*620;

      // V_l_low = V_l % 256;
      // V_l_high = V_l / 256;
      // V_r_low = V_r % 256;
      // V_r_high = V_r / 256;
      //四轮速度
      g_mc_ctrl_string[7] = g_mc_ctrl_string[11] = right_low;
      g_mc_ctrl_string[8] = g_mc_ctrl_string[12] = right_high;
      g_mc_ctrl_string[9] = g_mc_ctrl_string[13] = left_low;
      g_mc_ctrl_string[10] = g_mc_ctrl_string[14] = left_high;

      //计算速度校验位
      int sum;
      sum = 622+(right_low+right_high)*2+(left_low+left_high)*2;
      g_mc_ctrl_string[19] = sum % 256;
      g_mc_ctrl_string[20] = sum / 256;
      
      //发送底盘数据
      sleep(1);
      
      
      ntzx_usart_send(g_usart_fd,(char*)g_mc_ctrl_string, 24);
      // cout<<"rt="<<rt<<endl;
   
    }

}
void* thread_uasrtrecv(void* args){
    while(1){
      if (g_usart_fd < 0) {
      sleep(1);
      thread_uasrtrecv(args);
    };
    
    
    //接收底盘信息
    int i;int rt = -1;
    unsigned char string_recv[128] = {0};
    unsigned char *q = &string_recv[0];
    string recv_data;
    for (i = 0; i < 128; i++) {
      ntzx_usart_recv(g_usart_fd, (char*)q ,1);
      q++;
    }
    rt = ntzx_recv_data_handle(string_recv);
    recv_data=to_string(string_recv[0])+" "+to_string(string_recv[1])+" "+to_string(string_recv[2])+" "+
    to_string(string_recv[3])+" "+to_string(string_recv[4])+" "+to_string(string_recv[5])+" "+
    to_string(string_recv[6])+" "+to_string(string_recv[7])+" "+to_string(string_recv[8])+" "+
    to_string(string_recv[9])+" "+to_string(string_recv[10])+" "+to_string(string_recv[11])+" "+
    to_string(string_recv[12])+" "+to_string(string_recv[13])+" "+to_string(string_recv[14])+" "+
    to_string(string_recv[15])+" "+to_string(string_recv[16])+" "+to_string(string_recv[17])+" "+
    to_string(string_recv[18])+" "+to_string(string_recv[19])+" "+to_string(string_recv[20])+" "+
    to_string(string_recv[21])+" "+to_string(string_recv[22])+" "+to_string(string_recv[23])+" "+
    to_string(string_recv[24])+" "+to_string(string_recv[25]);
    // cout<<"string_recv="<<recv_data<<endl;
    if (rt >= 0) {
      cout<<"data recv success"<<endl;
      
    }
    else{cout<<"data recv fail"<<endl;}
    // cout<<rt<<endl;
    }
}





int main(int argc, char **argv)
{
    pthread_t id1,id2;
    
    //创建串口接收、发送线程
    pthread_create( &id1, NULL, thread_uasrtsend, NULL );
    pthread_create( &id2, NULL, thread_uasrtrecv, NULL );
    // pthread_join(id,NULL);
    rclcpp::init(argc, argv);
    
    /*产生一个Wang2的节点*/
    auto node = std::make_shared<TopicSuscribe01>("topic_subscribe_01");
  // 打印一句自我介绍
  
    RCLCPP_INFO(node->get_logger(), "example_fish_protocol节点已经启动.");
    /* 运行节点，并检测退出信号*/
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}