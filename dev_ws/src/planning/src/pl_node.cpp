#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "planning/ntzx_planning_lane.hpp"
#include "planning/ntzx_qianxun_inv.hpp"
#include "planning/ntzx_tools.hpp"
#include "planning/ntzx_mc.hpp"
#include "planning/ntzx_timer_drv.hpp"
#include "planning/ntzx_conf_app.hpp"
#include "planning/ntzx_log_app.hpp"
#include "message_interfaces/msg/gnss.hpp"
#include "message_interfaces/msg/pl.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include<unistd.h>
#include<errno.h>
#include <string>

#define CAR_GRID_SIZE (10)     //车体栅格大小cm
#define deg_rad (0.01745329252)  //弧度转化成度
#define AHEAD_LENGTH (5)      //寻找目标点距离阈值
#define START_POSITION_X_DEVIATION_MAX  (2)   //偏航范围阈值,m  
#define START_POSITION_Y_DEVIATION_MAX  (3)
#define CAR_FRONT_OFFET (11)   //
#define COMPENSATE_TURN (1.5)
#define COMPENSATE_HEADING (1)
#define TURN_SPEED (50)     
#define HEADING_SPEED (80)  
#define RAD2DEG  (57.295779513)  
#define THE_NUM_OF_POINTS_TO_DETECT  (20)

#define MAX(a,b) (a>b)?a:b
// PL路径生成需求参数
////////////////////////////////////////////////////////////////////////////////////////////////
static str_ntzx_vehicle_waypoint g_road_info_pl = {0};
static str_inv_to_pl inv_to_pl = {0};
static str_ntzx_pl_to_mc g_pl_to_mc = {0};
static int g_local_exist_grid =1;
struct_mc_ctrl gg_mc = {0};

static double LastCarSpeed = 50;
static double LLastCarSpeed = 50;
static double LLLastCarSpeed = 50;

 /*将停车指令赋值结构体*/
static void ntzx_stop_car(void);  
static void ntzx_direction_control(void);
// int Road_Planning_Find_Nearest_Point(int start_index , str_inv_to_pl Cur_GPS_info,str_ntzx_vehicle_waypoint* Road_info);
int Road_Planning_Find_Aim_Point(int start_index, int  dest_num,str_inv_to_pl Cur_GPS_info,str_ntzx_vehicle_waypoint* Road_info);
static int Ntzx_Obstacle_Road_Detect(int nearest_point_index, int n_point_need_to_detect,str_inv_to_pl Cur_GPS_info,str_ntzx_vehicle_waypoint* Road_info);
str_ntzx_pl_to_mc  Get_MC_Param(int dest_index, str_inv_to_pl inv_to_pl,str_ntzx_vehicle_waypoint* g_road_info_pl);
int Generate_Road_Point_Info();


//找到匹配车辆当前位置。
int Road_Planning_Find_Nearest_Point(int start_index , str_inv_to_pl Cur_GPS_info,str_ntzx_vehicle_waypoint* Road_info)
{  
     if (start_index <0)
    {   
        return -1;
    }     
    int Nearest_point_index =0;
    float aft_distance =0,pre_distance =0;
    pre_distance = ntzx_GPS_length(Cur_GPS_info.lon,Cur_GPS_info.lat,Road_info->waypoint_info[start_index].Longitude_degree,Road_info->waypoint_info[start_index].Latitude_degree);
    
    int start = start_index+1;
    for (int i = start; i < (Road_info->num-2); i++)
    {   
           
        aft_distance = ntzx_GPS_length(Cur_GPS_info.lon,Cur_GPS_info.lat,Road_info->waypoint_info[i].Longitude_degree,Road_info->waypoint_info[i].Latitude_degree);
        if ( aft_distance<pre_distance )
        {
            printf("%f",aft_distance);
            pre_distance = aft_distance;
            printf("%f",pre_distance);
            continue;
        }else{
            Nearest_point_index =i-1;
            printf("匹配点下标：%d ;  ",Nearest_point_index);
            printf("当前与匹配点相距：%f m\n  ",pre_distance);
            printf("%f,%f\n",Cur_GPS_info.lon,Cur_GPS_info.lat);
            return Nearest_point_index;
        }       
    } 
    return -1234; 

}






class TopicPublisher01 : public rclcpp::Node
{
public:
  TopicPublisher01(std::string name) : Node(name)
    {
        //RCLCPP_INFO(this->get_logger(),"我是%s,发布话题为:/command.",name.c_str());
        gnss_subscriber_ = this->create_subscription<message_interfaces::msg::Gnss>("gnss_command", 10, std::bind(&TopicPublisher01::topic_callback, this, std::placeholders::_1));
        pl_publisher_ = this->create_publisher<message_interfaces::msg::Pl>("pl_command",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&TopicPublisher01::timer_callback,this));
    }
private:
    void topic_callback(const message_interfaces::msg::Gnss::SharedPtr msg)
        {
           RCLCPP_INFO(this->get_logger(),"courseAngle:%s\tlat:%s\tlon:%s\tspeedmm:%s",msg->courseangle.c_str(),msg->lat.c_str(),msg->lon.c_str(),msg->vel_n.c_str());
           inv_to_pl.courseAngle = stof(msg->courseangle);
           inv_to_pl.lat = stof(msg->lat);
           inv_to_pl.lon = stof(msg->lon);
           inv_to_pl.speedmm = stof(msg->vel_n);
           printf("接收后：\ncourseAngle:%f\tlat:%f\tlon:%f\tspeedmm:%f",inv_to_pl.courseAngle,inv_to_pl.lat,inv_to_pl.lon,inv_to_pl.speedmm);
        };
    
   void timer_callback()
    {
        message_interfaces::msg::Pl message;

        int road_point_nearest_index = 0;
        int road_point_need_to_march_index=1;
        ntzx_systimer plan_sys = {0};
        /* 获取路径点 */
        ntzx_get_vehicle_waypoint(&g_road_info_pl);
        Timer_Set(&plan_sys, 50000);
        printf("planning start");
        while (1) 
        {  //执行频率20hz
            if (Timer_GetReached(&plan_sys) < 0) 
                continue;
            Timer_Set(&plan_sys, 50000);
            /*1.实时获取导航数据，m emcpy*/
            // int rt = ntzx_get_inv_to_pl(&inv_to_pl);
            if (inv_to_pl.lat == 0.0 || inv_to_pl.lon == 0.0) {
                printf("等待有效GPS数据\n");
                sleep(2);
                qianxun_flush();
                ntzx_stop_car();
                continue;
            }
            // printf("get gps data");
            /*2. 寻找当前路径下跟当前定位最近的点*/
            road_point_nearest_index = Road_Planning_Find_Nearest_Point(road_point_nearest_index,inv_to_pl,&g_road_info_pl);
            if (road_point_nearest_index<0){
                printf("路径点读取结束!%d\n",road_point_nearest_index);
                while (1);
            } 
            // printf("find the point1");      
            /*3.找到车体前方大于50cm，且在车子周身2m*3m的区域范围内的路径点下标*/
            road_point_need_to_march_index = Road_Planning_Find_Aim_Point(road_point_nearest_index,AHEAD_LENGTH,inv_to_pl,&g_road_info_pl);
            if ( road_point_need_to_march_index < 0) {
                printf(" 车体偏离路径距离过大！\n");
                continue;
            }
            // printf("find the point2");
            /*4.检测道路区域是否有障碍物*/
            // g_local_exist_grid = Ntzx_Obstacle_Road_Detect(road_point_nearest_index, THE_NUM_OF_POINTS_TO_DETECT,inv_to_pl,&g_road_info_pl);
            // if (g_local_exist_grid !=0)//检测到障碍物
            // {   
            //     if (g_local_exist_grid ==1)
            //     {
            //         printf(" #########################      检测到障碍物！        #########################！\n");
            //         ntzx_stop_car();
            //         ntzx_direction_control();
            //         continue;
            //     }else
            //     {
            //         printf(" 未能读取有效雷达数据\n");
            //         continue;
            //     }             
            // }
            // printf("-----无障碍物！-----\n");
            /*5.      计算mc参数       */
            g_pl_to_mc = Get_MC_Param(road_point_need_to_march_index,inv_to_pl,&g_road_info_pl);
            /*6.      下发控制指令    */
            printf("courseAngle=%d,speed_cm_ps=%d,earthCoordX_cm=%d,earthCoordY_cm=%d\n",g_pl_to_mc.courseAngle,g_pl_to_mc.speed_cm_ps,g_pl_to_mc.earthCoordX_cm,g_pl_to_mc.earthCoordY_cm);
            ntzx_direction_control();
            
            printf("ctrl success");
        }

        
        message.speed_mm = std::to_string(gg_mc.speed_mm);
        message.steer_angle = std::to_string(gg_mc.steer_angle);
        RCLCPP_INFO(this->get_logger(),"speed_mm:%s\tsteer_angle:%s",message.speed_mm.c_str(),message.steer_angle.c_str());
        
        pl_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<message_interfaces::msg::Gnss>::SharedPtr gnss_subscriber_;
    rclcpp::Publisher<message_interfaces::msg::Pl>::SharedPtr pl_publisher_;

};

//计算MC参数
 str_ntzx_pl_to_mc  Get_MC_Param(int dest_index, str_inv_to_pl inv_to_pl,str_ntzx_vehicle_waypoint* g_road_info_pl)
{   
    str_ntzx_pl_to_mc mc_Param;
    static int angle_Compensation=0,angle_Compensation_last=0,angle_Compensation_llast=0;
    int angle_Now = inv_to_pl.courseAngle;
    float ki_fixed;
    double point_x,point_y;
    int angle_Dest = g_road_info_pl->waypoint_info[dest_index].courseAngle;
    //将修正角度始终控制在+/-180度。
    if(angle_Dest - angle_Now >180){
        angle_Now += 360;
    }
    if(angle_Dest - angle_Now <-180){
        angle_Dest += 360;
    }
        //航向角的补偿
    angle_Compensation = (angle_Dest - angle_Now);
        //转向时候补偿角系数变小
    if(abs(angle_Compensation_llast-angle_Compensation_last)>=1 && abs(angle_Compensation_last-angle_Compensation)>=1){
        ki_fixed = COMPENSATE_TURN;
        printf("进入转弯 ！\n");
    }else{
        ki_fixed = COMPENSATE_HEADING;
    }
    ntzx_GPS_posit(inv_to_pl.courseAngle,inv_to_pl.lon,inv_to_pl.lat,g_road_info_pl->waypoint_info[dest_index].Longitude_degree,g_road_info_pl->waypoint_info[dest_index].Latitude_degree,&point_x,&point_y);
    //printf("目标点相对位置x：%f,y:%f\n",point_x,point_y);

    mc_Param.courseAngle = (int)(atan(point_x / point_y) *RAD2DEG)+(int)(ki_fixed*angle_Compensation); //单位是角度
    printf("转向角度计算：%d\n",mc_Param.courseAngle);
    mc_Param.drvCmd = NTZX_MC_ACTION_GO_FORWARD;
    mc_Param.earthCoordX_cm = 0;
    mc_Param.earthCoordY_cm = 0;
    if (ki_fixed== COMPENSATE_TURN)
    {
        mc_Param.speed_cm_ps = TURN_SPEED;
    }  
        mc_Param.speed_cm_ps = HEADING_SPEED;

    angle_Compensation_llast = angle_Compensation_last;
    angle_Compensation_last = abs(g_pl_to_mc.courseAngle);  //angle_Compensation;
    return mc_Param;
}

int Generate_Road_Point_Info()
{

}



//找到要前进的点，离车子大于0.5m,范围是左右3m，前后4m。
int Road_Planning_Find_Aim_Point(int start_index, int  dest_num,str_inv_to_pl Cur_GPS_info,str_ntzx_vehicle_waypoint* Road_info)
{   
    double x_diff=0,y_diff=0;
    int dest_index = start_index;
    if (start_index+dest_num <=(Road_info->num-1))
    {
        dest_index = start_index+dest_num;
    }
    else
    {
        dest_index = (Road_info->num-1);
    }  
    ntzx_GPS_posit(Cur_GPS_info.courseAngle,Cur_GPS_info.lon,Cur_GPS_info.lat,Road_info->waypoint_info[dest_index].Longitude_degree,Road_info->waypoint_info[dest_index].Latitude_degree,&x_diff,&y_diff);
    //printf("目标点偏差：：x=%f,y=%f \n",x_diff,y_diff);
    if ( abs(x_diff)>START_POSITION_X_DEVIATION_MAX ||  abs(y_diff>START_POSITION_Y_DEVIATION_MAX ))
    {  
        return -1;
    } 
    return (dest_index);
}


/* 判断当前坐标系下后续的障碍点 */
static void ntzx_stop_car(void)
{
    memset(&g_pl_to_mc, 0, sizeof(str_ntzx_pl_to_mc));
    g_pl_to_mc.drvCmd = NTZX_MC_ACTION_STOP;
    g_pl_to_mc.speed_cm_ps = 0;
}

static void ntzx_direction_control(void)
{   
    gg_mc.action = g_pl_to_mc.drvCmd;
    if (gg_mc.action == NTZX_MC_ACTION_STOP) {
        if (gg_mc.speed_mm != 0) {
            gg_mc.speed_mm = (0 + 0 + 0 + 0) * 10 / 4;
        }
        gg_mc.io_ctrl = 0;
        LLLastCarSpeed = LLastCarSpeed;
        LLastCarSpeed = LastCarSpeed;
        LastCarSpeed = gg_mc.speed_mm;
        //ntzx_mc_ctrl(gg_mc);
        return;
    }
   gg_mc.speed_mm = (50 + LastCarSpeed + LLastCarSpeed + LLLastCarSpeed) / 3;
    // gg_mc.speed_mm =80;
    // gg_mc.io_ctrl = 0;
    // gg_mc.steer_angle = g_pl_to_mc.courseAngle;
    // LLLastCarSpeed = LLastCarSpeed;
    // LLastCarSpeed = LastCarSpeed;
    // LastCarSpeed = gg_mc.speed_mm;
    // ntzx_mc_ctrl(gg_mc);
}

static int Ntzx_Obstacle_Road_Detect(int nearest_point_index, int n_point_need_to_detect,str_inv_to_pl Cur_GPS_info,str_ntzx_vehicle_waypoint* Road_info)
{
    Grid_coordinate Car_Occupy_special[16][12];  //该数组代表小车栅格坐标，16行*12列，特殊情况下用。
    Grid_coordinate Car_Occupy[1][12];  //检测框模板
    Grid_coordinate Car_Occupy_temp[1][12]; //实际检测框
    int  rt,obstacle_x, obstacle_y,position;
    double x_diff,y_diff,angle_rad_diff;
    double rotate_k1,rotate_k2,rotate_k3,rotate_k4; 
    static str_grid_fuse_to_pl  lidar_data;
    //获取激光雷达数据
    rt=ntzx_get_grid_data(&lidar_data);
    if (rt < 0)
    {   
        //printf("激光雷达数据: %s(errno: %d)\n", strerror(errno), errno);
        return rt;
    }
    
    //初始化小车栅格坐标
    for(int row = 0 ;row <16;row++)
    {
        for (int  column = 0; column < 12; column++)
        {
            Car_Occupy_special[row][column].x = -55 + column*CAR_GRID_SIZE; //栅格点坐标取栅格正中间位置，左下角第一个栅格点坐标为（-55，5）；相邻栅格点横、纵坐标相差10cm
            Car_Occupy_special[row][column].y = 5 +row*CAR_GRID_SIZE;           
        }    
    }  
    for(int row = 0 ;row <1;row++)
    {
        for (int  column = 0; column < 12; column++)
        {
            Car_Occupy[row][column].x = -55 + column*CAR_GRID_SIZE; 
            Car_Occupy[row][column].y = 0 +row*CAR_GRID_SIZE;
        }    
    }  
    //检测代码主体
    //特殊情况：当车子偏离路线和快到路尽头时，只检测正前方1.5m的障碍物
    float distance = ntzx_GPS_length(Cur_GPS_info.lon,Cur_GPS_info.lat,Road_info->waypoint_info[nearest_point_index].Longitude_degree,Road_info->waypoint_info[nearest_point_index].Latitude_degree);
    if ((distance>0.3) ||  (nearest_point_index+CAR_FRONT_OFFET+n_point_need_to_detect) >=(Road_info->num-1) )  //车身长度为10个坐标点间距
    {	
        printf("?????????????????特殊情况????????????????????????????");
        for(int row = 0 ;row <16;row++)
        {
            for (int  column = 0; column < 12; column++)
            {
                obstacle_x = (Car_Occupy_special[row][column].x )+ LIDAR16_LEFT_CAR_DISTANCE_CM;  //x轴不变
                obstacle_y = LIDAR16_BREFORE_CAR_DISTANCE_CM - (Car_Occupy_special[row][column].y + 130); //检测从车屁股前130cm开始
                position = (obstacle_x / LIDAR_VERTICAL_DISTANCE_RESOLUTION_CM) + (obstacle_y / LIDAR_HORIZONTAL_DISTANCE_RESOLUTION_CM) * LIDAR_HORIZONTAL_GRID_NUM;		
                /* 判断汽车周身是否存在障碍物 */
                 if (lidar_data.gridmask[position] == WRC_3D16_OBS_SURE) {
                    return NTZX_FUSE_GRID_EXIST ;
                }  
            }
        }
      return NTZX_FUSE_GRID_NOEXIST;
    }
    else//一般情况
    {	
	    printf("一般情况\n");
        for ( int i = (nearest_point_index+CAR_FRONT_OFFET); i < (nearest_point_index+CAR_FRONT_OFFET+n_point_need_to_detect); i++)
        {
            //计算目标点的偏差
            ntzx_GPS_posit(Cur_GPS_info.courseAngle,Cur_GPS_info.lon,Cur_GPS_info.lat,Road_info->waypoint_info[i].Longitude_degree,Road_info->waypoint_info[i].Latitude_degree,&x_diff,&y_diff );

            //计算旋转矩阵参数
            angle_rad_diff = (Road_info->waypoint_info[i].courseAngle-Cur_GPS_info.courseAngle)*deg_rad;
            rotate_k1 = cos(angle_rad_diff);
            rotate_k2= (-1)*sin(angle_rad_diff);
            rotate_k3 =(-1)*rotate_k2;
            rotate_k4 = rotate_k1;
            printf("当前坐标：[%f,%f],目标坐标[%f,%f]\t",Cur_GPS_info.lon,Cur_GPS_info.lat,Road_info->waypoint_info[i].Longitude_degree,Road_info->waypoint_info[i].Latitude_degree);
            printf("第%d个目标点；与当前位置偏差：x=%fcm,y=%fcm，旋转角：%f/度---%f/rad^\n",
            i,x_diff*100,y_diff*100,Road_info->waypoint_info[i].courseAngle-Cur_GPS_info.courseAngle,angle_rad_diff);
            //计算实际模板坐标并检测
            //先旋转
            for(int row = 0 ;row <1;row++)
            {   
                // printf("/n旋转后:");  
                for (int  column = 0; column < 12; column++)
                {
                    Car_Occupy_temp[row][column].x = rotate_k1*Car_Occupy[row][column].x +rotate_k2*Car_Occupy[row][column].y ;
                    Car_Occupy_temp[row][column].y = rotate_k3*Car_Occupy[row][column].x +rotate_k4*Car_Occupy[row][column].y ;
                    // printf("[%d,%d]\t",Car_Occupy_temp[row][column].x,Car_Occupy_temp[row][column].y);                  
                 }
            }  
            //再平移
            for(int row = 0 ;row <1;row++)
            {   
                // printf("\n平移后:");  
                for (int  column = 0; column < 12; column++)
                {
                    Car_Occupy_temp[row][column].x =  Car_Occupy_temp[row][column].x+(x_diff*100);
                    Car_Occupy_temp[row][column].y =  Car_Occupy_temp[row][column].y+(y_diff*100);

                    // printf("[%d,%d]\t",Car_Occupy_temp[row][column].x,Car_Occupy_temp[row][column].y);
                    //将雷达位置转换成数组坐标
                    obstacle_x = Car_Occupy_temp[row][column].x + LIDAR16_LEFT_CAR_DISTANCE_CM;
                    obstacle_y = LIDAR16_BREFORE_CAR_DISTANCE_CM - Car_Occupy_temp[row][column].y;
                    position = (obstacle_x / LIDAR_VERTICAL_DISTANCE_RESOLUTION_CM) + 
                    (obstacle_y / LIDAR_HORIZONTAL_DISTANCE_RESOLUTION_CM) * LIDAR_HORIZONTAL_GRID_NUM;
                    
                    /* 判断汽车周身是否存在障碍物 */
                    if (lidar_data.gridmask[position] == WRC_3D16_OBS_SURE)
                    {
                        return NTZX_FUSE_GRID_EXIST;
                    }                      
                }
                    
            }  

        }
        return NTZX_FUSE_GRID_NOEXIST;   
    } 
}


int main(int argc, char** argv) {

  /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    
    /*产生一个pl_node的节点*/
    auto node = std::make_shared<TopicPublisher01>("pl_publisher");
    // 打印一句自我介绍
    
    RCLCPP_INFO(node->get_logger(), "pl_publisher节点已经启动.");
   
    
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
    }