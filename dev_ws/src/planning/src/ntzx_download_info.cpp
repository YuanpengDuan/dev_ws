/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_download_info.c
 * 作者：任家豪
 * 日期：2021年2月22日
 * 描述：平台数据下发功能
 * ******************************************************************/
#include "planning/ntzx_download_info.hpp"
#include "planning/ntzx_log_app.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#define TEST_LOAD   "gps_load.txt"
static str_ntzx_vehicle_waypoint g_ntzx_vehicle_waypoint = {0};
static pthread_mutex_t g_mutex_ntzx_vehicle_waypoint = PTHREAD_MUTEX_INITIALIZER;

str_ntzx_vehicle_waypoint ntzx_download_info_test(void)
{
    FILE *fp = NULL;
    char tempbuf[24] = {0};
    int i = 0;
    fp = fopen("gps_load.txt", "r");
    printf("fp=%f\n",fp);
    if (fp == NULL) {
        ntzx_stereotypes_log_write_buf("can not open gps_load.txt");
        // return NTZX_DOWNLOAD_OPEN_ERR;
    }
    while (!feof(fp)) 
    {
          if(NULL != fgets(tempbuf, sizeof(tempbuf), fp)) {
            g_ntzx_vehicle_waypoint.waypoint_info[i].Latitude_degree = atof(tempbuf);//GPS接收机经纬度数据是真实数据的100倍数
        }
        if(NULL != fgets(tempbuf, sizeof(tempbuf), fp)) {
            g_ntzx_vehicle_waypoint.waypoint_info[i].Longitude_degree = atof(tempbuf);
        }
        if(NULL != fgets(tempbuf, sizeof(tempbuf), fp)) {
            g_ntzx_vehicle_waypoint.waypoint_info[i].courseAngle = atof(tempbuf);
        }
        if(NULL != fgets(tempbuf, sizeof(tempbuf), fp)) {
            g_ntzx_vehicle_waypoint.waypoint_info[i].speed_cm = atoi(tempbuf);
        }
        i++;
        printf("iiii=%d\n",i);
    }
    i--;
    printf("i=%d\n",i);
    g_ntzx_vehicle_waypoint.num = i;
    // return NTZX_DOWNLOAD_SUCCESS;
    return g_ntzx_vehicle_waypoint;
}

void *ntzx_download_info_main(void* arg)
{

}

void ntzx_get_vehicle_waypoint(str_ntzx_vehicle_waypoint *waypoint)
{
    printf("enter");
    pthread_mutex_lock(&g_mutex_ntzx_vehicle_waypoint);
    memcpy(waypoint, &g_ntzx_vehicle_waypoint, sizeof(str_ntzx_vehicle_waypoint));
    pthread_mutex_unlock(&g_mutex_ntzx_vehicle_waypoint);
}
