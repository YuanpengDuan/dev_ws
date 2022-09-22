/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_tools.c
 * 作者：任家豪
 * 日期：2021年2月26日
 * 描述：一些常用的工具
 * ******************************************************************/
#include "planning/ntzx_tools.hpp"
#include <math.h>
#include<unistd.h>
#include <stdio.h>
/////////////////////////////////////////////////////////////////////////////////////
//
// 坐标转换
//
/////////////////////////////////////////////////////////////////////////////////////
//经纬度转地面平面坐标
// #define Lat_Origin   32.045062  		      // 地面平面坐标系原点的纬度，南京
// #define Lon_Origin    118.845200366   // 地面平面坐标系原点的经度
#define Lat_Origin   32.030859477  		      // 地面平面坐标系原点的纬度，南通
#define Lon_Origin    120.915069439   // 地面平面坐标系原点的经度
#define Re  6378137
#define Rn  6356755
#define deg_rad  (0.01745329252 )    // Transfer from angle degree to rad
#define R_LATI   (6378137)
#define R_LONT  (5407872)  //这个需要根据所在区域的纬度进行换算：R_LONT = R_LATI*cos(所在地的纬度转化为弧度)
int ntzx_Conver_E_N(double Longitude_degree, double Latitude_degree, double *EarthRefCoord_x_m, double *EarthRefCoord_y_m)
{

    *EarthRefCoord_y_m = (Latitude_degree - Lat_Origin)*Re*deg_rad;
    *EarthRefCoord_x_m = (Longitude_degree - Lon_Origin)*Re*deg_rad*cos(Lat_Origin*deg_rad);
    return 0;
}
//地面平面坐标转车体坐标系坐标
int ntzx_Conver_N_B(double in_yaw_rad, double TargetEarthCoord_x_m, double TargetEarthCoord_y_m, double CarEarthCoord_x_m, double CarEarthCoord_y_m, int *out_Coord_B_x_cm, int *out_Coord_B_y_cm)
{

    double s_psi = sin(in_yaw_rad);
    double c_psi = cos(in_yaw_rad);

    *out_Coord_B_x_cm = (int)(((-TargetEarthCoord_y_m*s_psi + TargetEarthCoord_x_m*c_psi) - (-CarEarthCoord_y_m*s_psi + CarEarthCoord_x_m*c_psi)) * 100);
    *out_Coord_B_y_cm = (int)(((TargetEarthCoord_y_m*c_psi + TargetEarthCoord_x_m*s_psi) - (CarEarthCoord_y_m*c_psi + CarEarthCoord_x_m*s_psi)) * 100);

    return 0;
}

//返回单位：m
double ntzx_GPS_length(double lonti1,double lati1,double lonti2,double lati2)
{
    double x=0,y=0;
    double length=0;

    x = (R_LONT)*(lonti2-lonti1)*deg_rad;  //弧长公式
    y = (R_LATI)*(lati2-lati1)*deg_rad;
    length = (double)sqrt((double)(x*x+y*y));
    // printf("lon2=%f,lon1=%f,lat2=%f,lat1=%f",lonti2,lonti1,lati2,lati1);
    // printf("%f",length);
    return length;

}

//Cur_navAngle指定y轴正方向.Cur_lonti，Cur_lati为原点；x_diff,y_diff是Dest_lonti,Dest_lati在该坐标系的直角坐标:m
int ntzx_GPS_posit(double Cur_navAngle,double Cur_lonti,double Cur_lati,double Dest_lonti,double Dest_lati,double *x_diff,double *y_diff)
{
    double Navi_rad,x,y;
    float k1,k2,k3,k4; //旋转矩阵的四个参数，对应cos(theta),-sin(theta),sin(theta),cos(theta);
    Navi_rad = Cur_navAngle*deg_rad;//角度转化为弧度
    //以当前位置为原点，以正北为y正轴，以正东为x正轴；
    x = (Dest_lonti-Cur_lonti)*deg_rad*R_LONT;
    y = (Dest_lati- Cur_lati)*deg_rad*R_LATI;
    k1 = cos(Navi_rad);
    k2 = (-1)*sin(Navi_rad);
    k3 = (-1)*k2;
    k4 = k1;
    //以当前航向角作为旋转角
    *x_diff =  x*k1 + y*k2;
    *y_diff =  x *k3 + y*k4;

    return 0;
}

int Break_stop()
{
    sleep(2);
    while(1);
}