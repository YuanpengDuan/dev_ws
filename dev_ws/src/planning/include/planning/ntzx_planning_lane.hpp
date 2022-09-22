#ifndef NTZX_PLANNING_LANE_H_
#define NTZX_PLANNING_LANE_H_
#include "ntzx_download_info.hpp"
#include "ntzx_fuse_data.hpp"

#define NTZX_PLANNING_ERR   (-91)

#define NTZX_MAX_PL_PATH_POINTS_NUM      20       //规划路径的最多点数

typedef struct pl_path_point
{
    short  x_cm;        //点的X坐标,车底坐标系
    short  y_cm;        //点的Y坐标,车底坐标系
    short  speed_cm_ps;//该点的速度,单位:厘米/秒
} str_ntzx_pl_path_point;

//小车栅格点结构体
typedef struct grid_coordinate{
    int x;
    int y;
}Grid_coordinate;

typedef struct pl_to_mc 
{
    char drvCmd; //动作命令 见"动作"
    short  IO_Cmd; // IO控制命令  
    char nPoint; //有效点个数
    short speed_cm_ps; //规划速度,单位:厘米/秒
    str_ntzx_pl_path_point pPoint[NTZX_MAX_PL_PATH_POINTS_NUM]; //车体坐标系下的路径点
    int courseAngle; //航向角,角度值
    int earthCoordX_cm;     //  北向  +X 
    int earthCoordY_cm;     //  东向  +Y
} str_ntzx_pl_to_mc;

void *ntzx_plan_lane_main(void *arg);


#endif