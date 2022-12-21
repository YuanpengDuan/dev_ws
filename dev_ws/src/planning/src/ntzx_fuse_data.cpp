/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_qianxun_inv.h
 * 作者：任家豪
 * 日期：2021年2月26日
 * 描述：数据融合功能函数
 * ******************************************************************/
#include "planning/ntzx_fuse_data.hpp"
#include "planning/ntzx_log_app.hpp"
#include "planning/ntzx_lidar_leishen.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

static str_lidar_info_to_fuse g_fuse_lidar = {0};
static str_grid_fuse_to_pl g_grid_fuse_to_pl = {0};
static int g_proc_flag = 0;
static pthread_mutex_t g_mutex_to_pl = PTHREAD_MUTEX_INITIALIZER;//用于控制收发UDP包的线程

static void *ntzx_fuse_data_recv(void *arg);
static void *ntzx_fuse_data_proc(void *arg);

void *ntzx_fuse_data_main(void *arg)
{
    int rt;
    /* 创建两个线程，一个接收线程，一个处理线程 */
    pthread_t fuse_data_recv;
    pthread_t fuse_data_pro;
    rt = pthread_create(&fuse_data_recv, NULL, ntzx_fuse_data_recv, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("fuse creat recv thread err!!!");
    }
    rt = pthread_create(&fuse_data_pro, NULL, ntzx_fuse_data_proc, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("fuse creat recv thread err!!!");
    }
    pthread_join(fuse_data_recv, NULL);
    pthread_join(fuse_data_pro, NULL);
}

static void *ntzx_fuse_data_recv(void *arg)
{
    int rt = 0;
    str_lidar_info_to_fuse fuse_lidar = {0};
    while (1) {
        /* 获取激光雷达数据 */
        rt = ntzx_lidar_leishen_get_data(&fuse_lidar);
        if (rt < 0) {
            usleep(1000);
            continue;
        }
        while (g_proc_flag != 0) {

        }
        memcpy(&g_fuse_lidar, &fuse_lidar, sizeof(str_lidar_info_to_fuse));
        g_proc_flag = 1;
        usleep(1000);
    }
}

static void *ntzx_fuse_data_proc(void *arg)
{
    int frame_id = 0;
    while (1) {
        if (g_proc_flag == 0) {
            usleep(1000);
            continue;
        }
        /* 融合处理，由于该系统只有一个激光雷达判断所以没有融合部分代码 */

        /* 数据写入 */
        frame_id++;
        g_grid_fuse_to_pl.frame_id = frame_id;
        pthread_mutex_lock(&g_mutex_to_pl);
        memcpy(g_grid_fuse_to_pl.gridmask, g_fuse_lidar.gridmask, LIDAR_VERTICAL_GRID_NUM*LIDAR_HORIZONTAL_GRID_NUM);
        pthread_mutex_unlock(&g_mutex_to_pl);
        g_proc_flag = 0;
    }
}

/* 获取当前雷达数据点 */
int ntzx_get_grid_data(str_grid_fuse_to_pl *fuse_to_pl)
{
    printf("g_grid_fuse_to_pl.frame_id = %d",g_grid_fuse_to_pl.frame_id);
    if (fuse_to_pl->frame_id != g_grid_fuse_to_pl.frame_id) 
    {
        if(pthread_mutex_trylock(&g_mutex_to_pl) ==0) //成功加锁
        {
            memcpy(fuse_to_pl, &g_grid_fuse_to_pl, sizeof(str_grid_fuse_to_pl));
            pthread_mutex_unlock(&g_mutex_to_pl);           
            return NTZX_FUSE_SUCCESS;
        }      
       else{
           return NTZX_FUSE_TRYLOCK_FAIL;
       }       
    } 
    else {
        return NTZX_FUSE_INCOMPLETE;
    }
}