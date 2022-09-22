/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_lidar_leishen.h
 * 作者：李进
 * 日期：2021年2月26日
 * 描述：3D激光雷达数据的处理和接收
 * ******************************************************************/
#ifndef NTZX_LIDAR_LEISHEN
#define NTZX_LIDAR_LEISHEN

#define NTZX_LIDAR_LEISHEN_SUCCESS  0
#define NTZX_LIDAR_LEISHEN_PARA_ERR (-71)
#define NTZX_LIDAR_LEISHEN_INIT_ERR (-72)
#define NTZX_LIDAR_LEISHEN_BIND_ERR (-73)
#define NTZX_LIDAR_LEISHEN_CREATE_THREAD_ERR (-74)
#define NTZX_LIDAR_LEISHEN_DATA_ERR (-74)

#define LIDAR_LEISHEN_UDP_PACKET_SIZE   1206 // UDP包的大小
#define LIDAR_MAX_UDP_PACKET_SIZE   1348     // UDP包的最大大小
#define LIDAR_LENGTH_OF_FIRST_PART_OF_ONE_BLOCK  52 //一个数据块中第一个部分的长度，48B+2B+2B=52B
#define LIDAR_LENGTH_OF_SECOND_PART_OF_ONE_BLOCK 48 //一个数据块中第二个部分的长度
#define LIDAR_PACKET_TAIL_LEN 6  //一个udp包的尾部有6个字节
#define LIDAR_MAX_ONE_FRAME_GROUP_NUM  2000 //可能出现段错误，每帧数据（一圈，水平360度）包含2000 组垂直方向的数据（每组16个，对应16线激光发射器的数据）。
#define LIDAR_ONE_ROUND_ANGLE_NUM 36000 //一圈最大的角度数
#define LIADR_MAX_ONE_FRAME_PACKET  8400    //一帧100组/一包24组=41.66，一帧最多由42个UDP包拼成，这里设置4200
#define LIDAR_PACKET_INTERVAL_TIME (50)//(//1200)//按雷达每一组耗时50us 一包共有24组数据 50*24=1200us/每包  两包之间1.2ms 设计延迟应该<1.2ms，这里设置240us

#define LIDAR_MAX_ONE_FRAME_POINT_NUM  (16 * LIDAR_MAX_ONE_FRAME_GROUP_NUM) //一帧图像中有效点最高个数,

#define PIDIVIDED    0.0174533  //   PI/180

#define NUM_OF_LASERS 16//雷达扫描线数
#define LIDAR_TRANSFORM_START_ANGLE 18000//重新组织顺序时，开始记录的角度

#define LIDAR_VERTICAL_DISTANCE_RESOLUTION_CM       10 // 雷达垂直距离的分辨率
#define LIDAR_HORIZONTAL_DISTANCE_RESOLUTION_CM     10 // 雷达水平距离的分辨率

#define LIDAR_MIN_DETECT_DISTANCE 25  //25cm内的点过滤掉
#define LIDAR_MAX_DETECT_DISTANCE 15000 //150m内的点过滤掉
#define LIDAR_HANGLE_RESOLUTION 0.18  //角分辨率0.36，对应20hz
#define LIDAR_VERTICAL_DISTANCE_CM      2000    // 雷达垂直距离监测范围划定，最好为分辨率的倍数
#define LIDAR_HORIZONTAL_DISTANCE_CM    1600    // 雷达水平距离监测范围划定，最好为分辨率的倍数

#define LIDAR_VERTICAL_GRID_NUM     (LIDAR_VERTICAL_DISTANCE_CM/LIDAR_VERTICAL_DISTANCE_RESOLUTION_CM)
#define LIDAR_HORIZONTAL_GRID_NUM   (LIDAR_HORIZONTAL_DISTANCE_CM/LIDAR_HORIZONTAL_DISTANCE_RESOLUTION_CM)

#define LIDAR_GRID_ALL_NUM  LIDAR_VERTICAL_GRID_NUM*LIDAR_HORIZONTAL_GRID_NUM // 雷达总栅格数大小

#define LIDAR16_BREFORE_CAR_DISTANCE_CM    1000//车前1000cm
#define LIDAR16_AFTER_CAR_DISTANCE_CM      1000//车后1000cm
#define LIDAR16_LEFT_CAR_DISTANCE_CM         800//车左800cm
#define LIDAR16_RIGHT_CAR_DISTANCE_CM      800//车右800cm

#define HEIGHT_THRESHOLD_MAX 200 //一个栅格中被判断为障碍物的最大高度阈值
#define HEIGHT_THRESHOLD_MIN  25//一个栅格中被判断为障碍物的最小高度阈值
#define HEIGHT_THRESHOLD    5//高度差阈值

#pragma pack(push, 1)
//内部结构体,代表某个角度一条垂直的竖线上16个点
typedef struct {
    int angle;//角度,取值范围[0,36000-1]
    int distance[16]; //单位cm
    float intense[16];//反射率
} GROUP;

//存储的3D点定义
typedef struct {
    int scanID;	//扫描线ID,[0,15]
    int angle; //角度,取值范围[0,36000-1]
    double x; //单位cm
    double y; //单位cm
    double z; //单位cm
    int d; //单位cm(边心距,Range Image)
    float intense; //回波强度
} lenshen_3D16_Point;

//障碍物类型
typedef enum  tagWRC_3D16_OBS_TYPE
{
    WRC_3D16_OBS_NONE = 0, //平地，非障碍物
    WRC_3D16_OBS_UNSURE=125,//不确定是否为障碍物
    WRC_3D16_OBS_SURE=250//确定为障碍物
} WRC_3D16_OBS_TYPE;

typedef struct lidar_info_to_fuse{
    int             lidar_id;       // 雷达ID号
    int             frame_id;       // 雷达点云图(帧号)
    unsigned char   gridmask[LIDAR_VERTICAL_GRID_NUM*LIDAR_HORIZONTAL_GRID_NUM]; // 栅格点数据
}str_lidar_info_to_fuse;
#pragma pack(pop)

void *ntzx_lidar_leishen_main(void *arg);
int ntzx_lidar_leishen_get_data(str_lidar_info_to_fuse *lidar_data);

#endif
