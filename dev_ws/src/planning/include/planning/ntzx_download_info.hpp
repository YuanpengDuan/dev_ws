/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_download_info.h
 * 作者：任家豪
 * 日期：2021年2月22日
 * 描述：平台数据下发功能的对外头文件
 * ******************************************************************/
#ifndef _NTZX_DOWNLOAD_INFO_H_
#define _NTZX_DOWNLOAD_INFO_H_

#define NTZX_DOWNLOAD_SUCCESS       0
#define NTZX_DOWNLOAD_OPEN_ERR      (-81)

#define NTZX_DOWNLOAD_MAX_INFO  50000

typedef struct ntzx_vehicle_waypoint_info {
    double Latitude_degree;  // 纬度，单位:度
    double Longitude_degree; // 经度，单位:度
    double courseAngle;      // 当前路径点的航向角
    double speed_cm;            // 当前点车速，cm/s
} str_ntzx_vehicle_waypoint_info;

typedef struct ntzx_vehicle_waypoint {
    int num;
    str_ntzx_vehicle_waypoint_info waypoint_info[NTZX_DOWNLOAD_MAX_INFO];
} str_ntzx_vehicle_waypoint;

void *ntzx_download_info_main(void* arg);
int ntzx_download_info_test(void);
void ntzx_get_vehicle_waypoint(str_ntzx_vehicle_waypoint *waypoint);

#endif