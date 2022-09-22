/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_tools.c
 * 作者：任家豪
 * 日期：2021年2月26日
 * 描述：一些常用的工具
 * ******************************************************************/

#ifndef _NTZX_TOOLS_H_
#define _NTZX_TOOLS_H_

#define PI 3.1415926 // 定义一个圆周率的值

int ntzx_Conver_E_N(double Longitude_degree, double Latitude_degree, double *EarthRefCoord_x_m, double *EarthRefCoord_y_m);
int ntzx_Conver_N_B(double in_yaw_rad, double TargetEarthCoord_x_m, double TargetEarthCoord_y_m, double CarEarthCoord_x_m, double CarEarthCoord_y_m, int *out_Coord_B_x_cm, int *out_Coord_B_y_cm);
double ntzx_GPS_length(double lonti1,double lati1,double lonti2,double lati2);
int ntzx_GPS_posit(double Cur_navAngle,double Cur_lonti,double Cur_lati,double Dest_lonti,double Dest_lati,double *x_diff,double *y_diff);
int Break_stop();


#endif