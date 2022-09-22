/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_qianxun_inv.c
 * 作者：任家豪
 * 日期：2021年2月22日
 * 描述：千寻惯导的初始化，以及一些对外封装
 * ******************************************************************/
#include "planning/ntzx_qianxun_inv.hpp"
#include "planning/ntzx_usart_drv.hpp"
#include "planning/ntzx_log_app.hpp"
#include "planning/ntzx_conf_app.hpp"
#include "planning/ntzx_timer_drv.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#define QIANXUN_TITLE   "NAV"
#define QIANXUN_USART_KEY   "USART_NAV"

/* 定义惯导盒子的物理接口 */
#define COMPORT1    "COM1"
#define COMPORT2    "COM2"

#define QIANXUN_RECV_DATA_LEN   512

/* 有关惯导配置的指令和相关宏定义 */
#define UNLOG_TYPE  0   // 清楚所有打印输出
#define GPYBM_TYPE  1   // 接收惯导的姿态信息
#define SAVE_TYPE   2   // 保存配置
#define GPGGA_TYPE  3   // 接收惯导信息
#define GPHDT_TYPE  4   // 接收航向角信息

/* 标志位 */
#define QIANXUN_GPGGA_SYNC_FLAG 0x01
#define QIANXUN_GPHDT_SYNC_FLAG 0x02

/* 数据同步完成 */
#define QIANXUN_DATA_SYNC_FALG (QIANXUN_GPGGA_SYNC_FLAG | QIANXUN_GPHDT_SYNC_FLAG)

#define UNLOG_CMD    "UNLOG"
#define GPYBM_CMD    "GPYBM"
#define SAVE_CMD     "SAVECONFIG"
#define GPGGA_CMD    "GPGGA"
#define GPHDT_CMD    "GPHDT"

#define RECV_GPYBM_HEAD  "$GPYBM"
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
static str_vel_data g_vel_data = {0};
static int g_qanxun_data_sync_flag = 0;
ntzx_systimer clean_timer = {0};
/* 数据锁 */
static pthread_mutex_t g_mutex_qianxun_gps_info = PTHREAD_MUTEX_INITIALIZER;

static int ntzx_qianxun_usart_init(void);
static int ntzx_qianxun_send(char *recv_buf, int recv_len);
static int ntzx_qianxun_config_judge(void);
static int ntzx_qianxun_config(int cmd_type, float upload_v);
static int ntzx_gpgga_proc(void);
static int ntzx_gphdt_proc(void); 
static int ntzx_gpybm_proc(void);
static int ntzx_qianxun_recv(char *recv_buf, int recv_len);
static void *ntzx_qianxun_inv_recv(void *arg);
static void *ntzx_qianxun_inv_proc(void *arg);
static void ntzx_qianxun_restart(void);


void *ntzx_qianxun_inv_main(void *arg)
{
    
    int rt;
    pthread_t thread_inv_recv;
    pthread_t thread_inv_proc;
    /* 初始化导航 */
    rt = ntzx_qianxun_init();
    if (rt < 0) {
        ntzx_stereotypes_log_write_err("INV INIT:", rt);
        return NULL;
    }
    /* 写入千寻惯导初始化成功 */
    ntzx_stereotypes_log_write_buf("qianxun init success!!!");

    /* 创建惯导信息接收线程 */
    rt = pthread_create(&thread_inv_recv, NULL, ntzx_qianxun_inv_recv, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("creat inv recv thread err!!!");
        return NULL;
    }
    /* 创建惯导信息处理线程 */
    rt = pthread_create(&thread_inv_proc, NULL, ntzx_qianxun_inv_proc, NULL);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("creat inv recv thread err!!!");
        return NULL;
    }
    pthread_join(thread_inv_recv, NULL);
    pthread_join(thread_inv_proc, NULL);
    printf("prov and recv success");
}

/* 获取导航信息 */
int ntzx_get_inv_to_pl(str_inv_to_pl *nav_info)
{   
    g_gps_data.qual = NTZX_QIANXUN_QUAL_RTKFIX;
    if (  g_gps_data.qual == NTZX_QIANXUN_QUAL_RTKFIX   )
    {
        printf("enter inv");
        pthread_mutex_lock(&g_mutex_qianxun_gps_info);
        nav_info->courseAngle = g_gps_data.courseAngle;
        nav_info->lat = g_gps_data.lat;
        nav_info->lon = g_gps_data.lon;
        // nav_info->lat = g_gps_data.lat/100;//原始数据经纬度是实际经纬度的100倍，详细可用串口助手查看GPS原始数据
        // nav_info->lon = g_gps_data.lon/100;
        nav_info->speedmm = g_vel_data.Vel_N;//添加
        nav_info->frame = g_gps_data.frame;
        pthread_mutex_unlock(&g_mutex_qianxun_gps_info);
        printf("nav_lat=%f\n", nav_info->lat);
        return NTZX_QIANXUN_SUCCESS;
    }else {      
        printf("errrrrrr");
        return NTZX_QIANXUN_DATA_ERR;
    }      
    
}


/* 接收信息线程 */
static void *ntzx_qianxun_inv_recv(void *arg)
{
    char recv_buf[QIANXUN_RECV_DATA_LEN] = {0};
    int timeout_time = 0;
    int init_flag = 0; // 由于惯导热启动和冷启动时间问题，所以第一次上数据时间不超时
    /* 接收惯导数据 */
    while (1) {
        if (g_ntzx_qianxun_ufd > 0) {
            if (ntzx_qianxun_recv(recv_buf, sizeof(recv_buf)) < 0) {
                usleep(1000);
                /* 长时间没有收到数据则重启 */
                if (init_flag == 1) {
                    timeout_time++;
                    if (timeout_time == 10000) {
                        ntzx_qianxun_restart();
                        init_flag = 0;
                        timeout_time = 0;
                    }
                }
                continue;
            }
            timeout_time = 0;
            init_flag = 1;
            while (g_proc_flag != 0) {
                usleep(1000);
            }
            /* 最后一个参数+1是为将接收到的buf的最后以为\0字符也写入进去 */
            memcpy(g_qianxun_recv_data, recv_buf, strlen(recv_buf) + 1);
            g_proc_flag = 1;
            usleep(1000);
        }
    }
}

/* 处理信息线程 */
static void *ntzx_qianxun_inv_proc(void *arg)
{
    char *p = NULL;
    int rt;
    while (1) 
    {       
        if (g_proc_flag == 0) {
            usleep(1000);
            continue;
        }
        // /* GPGGA导航信息 */
        // p = strstr(g_qianxun_recv_data, QIANXUN_GPGGA_HEAD);
        // if (p != NULL) {
        //     rt = ntzx_gpgga_proc();
        //     if (rt < 0) {
        //         ntzx_stereotypes_log_write_err("GPGGA PROC ERR:", rt);
        //     }
        //     g_proc_flag = 0;
        //     continue;
        // }
        // /* GPHDT航向角信息 */
        // p = strstr(g_qianxun_recv_data, QIANXUN_GPHDT_HEAD);
        // if (p != NULL) {
        //     rt = ntzx_gphdt_proc();
        //     if (rt < 0) {
        //         ntzx_stereotypes_log_write_err("GPHDT PROC ERR:", rt);
        //     }
        //     g_proc_flag = 0;
        //     continue;
        // }
        /* 姿身信息 */
        p = strstr(g_qianxun_recv_data, RECV_GPYBM_HEAD);
        // printf("%s",g_qianxun_recv_data);
        if (p != NULL) {
            rt = ntzx_gpybm_proc();
            if (rt < 0) {
                ntzx_stereotypes_log_write_err("GPYBM GPYBM ERR:%s", rt);
            }
            g_proc_flag = 0;
            continue;
        }
        g_proc_flag = 0;
        qianxun_flush();
       
    }
}

// /* GPGGA处理函数 */
// static int ntzx_gpgga_proc(void)
// {
//     char *p_GPGGA = NULL;
//     char *q_GPGGA = NULL;
//     int time_GPGGA = 0;
//     int qual_GPGGA = 0;
//     int sats_GPGGA = 0;
//     float hdop_GPGGA = 0;
//     float alt_GPGGA = 0;
//     unsigned char a_unit_GPGGA = 0;
//     float undulation_GPGGA = 0;
//     unsigned char u_unit_GPGGA = 0;
//     unsigned char age_GPGGA = 0;
//     unsigned int stn_GPGGA = 0;
//     double lat = 0;
//     char lat_dir = 0;
//     double lon = 0;
//     char lon_dir = 0;
//     double courseAngle = 0;
//     int i = 0,j = 0;
//     for (; i < strlen(g_qianxun_recv_data); i++) {
//         if (g_qianxun_recv_data[i] == QIANXUN_SEPARATOR) {
//             j++;
//         }
//     }
//     if (j != 23) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     p_GPGGA = strchr(g_qianxun_recv_data, '*');
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     p_GPGGA = strchr(g_qianxun_recv_data, QIANXUN_SEPARATOR);
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     /*获取北京时间,获取到的时间格式为hhmmss.ss,小数点后的ss为毫秒*/
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     *q_GPGGA = 0;
//     time_GPGGA = (int)(atof(p_GPGGA) * 100);
//     /* 获取纬度 */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, QIANXUN_SEPARATOR);
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     lat = atof(q_GPGGA);
//     /* 获取是北纬还是南纬 */
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *q_GPGGA = 0;
//     lat_dir = (unsigned char)atoi(p_GPGGA);
//     /* 获取经度 */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     lon = atof(q_GPGGA);
//     /* 获取西经还是东经 */
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *q_GPGGA = 0;
//     lon_dir = (unsigned char)atoi(p_GPGGA);
//     /* 获取当前GPS、状态 */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, QIANXUN_SEPARATOR);
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     qual_GPGGA = (unsigned char)atoi(q_GPGGA);
//     /* 获取当前卫星数目 */
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *q_GPGGA = 0;
//     sats_GPGGA = (unsigned char)atoi(p_GPGGA);
//     /* 获取水平精度因子 */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, QIANXUN_SEPARATOR);
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     hdop_GPGGA = (float)atof(q_GPGGA);
//     /* 获取天线高度 */
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *q_GPGGA = 0;
//     alt_GPGGA = atof(p_GPGGA);
//     /* 获取天线高度单位 */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, QIANXUN_SEPARATOR);
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     a_unit_GPGGA = *q_GPGGA;
//     /* 获取大地水准面差距 */
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *q_GPGGA = 0;
//     undulation_GPGGA = atof(p_GPGGA);
//     /* 获取大地水准差距单位 */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, QIANXUN_SEPARATOR);
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     g_gps_data.u_unit = *q_GPGGA;
//     /* 获取差分数据龄期，单位为秒 */
//     p_GPGGA++;
//     q_GPGGA = strchr(p_GPGGA, QIANXUN_SEPARATOR);
//     if (q_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *q_GPGGA = 0;
//     age_GPGGA = (unsigned short)atoi(p_GPGGA);
//     /* 获取差分基站ID */
//     q_GPGGA++;
//     p_GPGGA = strchr(q_GPGGA, '*');
//     if (p_GPGGA == NULL) {
//         return NTZX_QIANXUN_ANALYSIS_ERR;
//     }
//     *p_GPGGA = 0;
//     stn_GPGGA = (unsigned short)atoi(q_GPGGA);

//     /*解析成功将数据写入结构体*/
//     pthread_mutex_lock(&g_mutex_qianxun_gps_info);
//     g_gps_data.time.hour = (unsigned char)(((time_GPGGA / 1000000) + 8) % 24); //+8是因为北京时间与世界时间差8个小时
//     g_gps_data.time.min = (unsigned char)((time_GPGGA % 1000000) / 10000);
//     g_gps_data.time.s = (unsigned char)((time_GPGGA % 10000) / 100);
//     g_gps_data.time.ms = (unsigned short)(time_GPGGA % 100) * 10;
//     g_gps_data.lat = lat;
//     g_gps_data.lat_dir = lat_dir;
//     g_gps_data.lon = lon;
//     g_gps_data.lon_dir = lon_dir;
//     g_gps_data.qual = qual_GPGGA;
//     g_qanxun_data_sync_flag |= QIANXUN_GPGGA_SYNC_FLAG;
//     pthread_mutex_unlock(&g_mutex_qianxun_gps_info);
//     return NTZX_QIANXUN_SUCCESS;
// }

/* GPTRA处理函数 */
static int ntzx_gpybm_proc(void)
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
    // printf("%s",dev_GPYBM);
    /*获取北京时间,获取到的时间格式为hhmmss.ss,小数点后的ss为毫秒*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    time_GPYBM = (int)(atof(q_GPYBM) * 100);
    // printf("time_GPYBM=%d",time_GPYBM);
    /*获取纬度信息*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    lon = atof(p_GPYBM);
    // printf("lon=%.3f",lon);
    /*获取经度信息*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    lat = atof(q_GPYBM);
    // printf("lat=%.3f",lat);
    /*获取椭球高*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    ElpHeight_GPYBM = (float)atof(p_GPYBM);
    // printf("ElpHeight_GPYBM=%.3f",ElpHeight_GPYBM);

    /*获取航向角*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Heading_GPYBM = atof(q_GPYBM);
    // printf("Heading_GPYBM=%.3f",Heading_GPYBM);

    /*获取俯仰角*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Pitch_GPYBM = atof(p_GPYBM);
    // printf("Pitch_GPYBM=%.3f",Pitch_GPYBM);

    /*获取北方向速度*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Vel_N_GPYBM = (float)atof(q_GPYBM);
    // printf("Vel_N_GPYBM=%.3f",Vel_N_GPYBM);

    /*获取东方向速度*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Vel_E_GPYBM = (float)atof(p_GPYBM);
    // printf("Vel_E_GPYBM=%.3f",Vel_E_GPYBM);

    /*获取地向速度*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Vel_D_GPYBM = (float)atof(q_GPYBM);
    // printf("Vel_D_GPYBM=%.3f",Vel_D_GPYBM);

    /*获取地面速度*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Vel_G_GPYBM = (float)atof(p_GPYBM);
    // printf("Vel_G_GPYBM = %.3f",Vel_G_GPYBM);
    
    /*获取高斯投影坐标X轴*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Coordinnate_Northing_GPYBM = atof(q_GPYBM);
    // printf("Coordinnate_Northing_GPYBM=%.3f",Coordinnate_Northing_GPYBM);

    /*获取高斯坐标Y轴*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Coordinnate_Easting_GPYBM = atof(p_GPYBM);
    // printf("Coordinnate_Easting_GPYBM=%f",Coordinnate_Easting_GPYBM);
    //////////////////
    /*获取基站坐标系下的移动站X坐标*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);

    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    North_Distance_GPYBM = atof(q_GPYBM);
    // printf("North_Distance_GPYBM=%.3f",North_Distance_GPYBM);
    /*获取基站坐标系下的移动站Y坐标*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    East_Distance_GPYBM = atof(p_GPYBM);
    // printf("East_Distance_GPYBM=%.3f",East_Distance_GPYBM);

    /*定位解状态*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Position_Indicator_GPYBM = (unsigned char)atoi(q_GPYBM);
    // printf("Position_Indicator_GPYBM=%d",Position_Indicator_GPYBM);

    /*定位解状态*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Heading_Indicator_GPYBM = (unsigned char)atoi(p_GPYBM);
    // printf("Heading_Indicator_GPYBM=%d",Heading_Indicator_GPYBM);

    /*主站天线收星数*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    SVn_GPYBM = (unsigned char)atoi(q_GPYBM);
    // printf("SVn_GPYBM=%d",SVn_GPYBM);

    /*差分延迟*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Diff_Age_GPYBM = (float)atof(p_GPYBM);
    // printf("Diff_Age_GPYBM=%.3f",Diff_Age_GPYBM);

    /*基准站ID*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Station_ID_GPYBM = (unsigned short)atoi(q_GPYBM);
    // printf("Station_ID_GPYBM=%d",Station_ID_GPYBM);

    /*主站到从站距离*/
    p_GPYBM++;
    q_GPYBM = strchr(p_GPYBM,QIANXUN_SEPARATOR);
    if (q_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *q_GPYBM = 0;
    Baseline_length_GPYBM = (float)atof(p_GPYBM);
    // printf("Baseline_length_GPYBM=%.3f",Baseline_length_GPYBM);

    /*从站参与解算的卫星数*/
    q_GPYBM++;
    p_GPYBM = strchr(q_GPYBM, QIANXUN_SEPARATOR);
    if (p_GPYBM == NULL) {
        return NTZX_QIANXUN_ANALYSIS_ERR;
    }
    *p_GPYBM = 0;
    Solution_sv_GPYBM = (unsigned short)atoi(q_GPYBM);
    // printf("Solution_sv_GPYBM=%d",Solution_sv_GPYBM);

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
    
    g_vel_data.Vel_N = Vel_N_GPYBM;     //速度
    g_vel_data.Vel_E = Vel_E_GPYBM;
    g_vel_data.Vel_G = Vel_G_GPYBM;
    g_vel_data.Vel_D = Vel_D_GPYBM;
    
    
    g_qanxun_data_sync_flag |= QIANXUN_GPGGA_SYNC_FLAG;
    pthread_mutex_unlock(&g_mutex_qianxun_gps_info);

    // printf("gps.lat=%f,gps.lon=%f,couseangle=%f,vel=%f\n",g_gps_data.lat,g_gps_data.lon,g_gps_data.courseAngle,g_vel_data.Vel_N);

    return NTZX_QIANXUN_SUCCESS;
}

// /* GPHDT处理函数 */
// static int ntzx_gphdt_proc(void)
// {
//     char *p_GPHDT = NULL;
//     char *q_GPHDT = NULL;
//     double courseAngle_GPHDT = 0;
//     q_GPHDT = strchr(g_qianxun_recv_data, QIANXUN_SEPARATOR);
//     if (q_GPHDT != NULL) {
//         q_GPHDT++;
//         p_GPHDT = strchr(q_GPHDT, QIANXUN_SEPARATOR);
//         if (p_GPHDT == NULL) {
//             return NTZX_QIANXUN_ANALYSIS_ERR;
//         }
//         *p_GPHDT = 0;
//         courseAngle_GPHDT = atof(q_GPHDT);
//     }
//     if (g_qanxun_data_sync_flag == QIANXUN_GPGGA_SYNC_FLAG) {
//         pthread_mutex_lock(&g_mutex_qianxun_gps_info);
//         g_gps_data.courseAngle = courseAngle_GPHDT;
//         g_gps_data_frame++;
//         g_gps_data.frame = g_gps_data_frame;
//         g_qanxun_data_sync_flag = 0;
//         pthread_mutex_unlock(&g_mutex_qianxun_gps_info);
//     } 
//     return NTZX_QIANXUN_SUCCESS;
// }

/* 千寻惯导配置函数 */
static int ntzx_qianxun_config(int cmd_type, float upload_v)
{
    int rt;
    char cmd_temp[100] = {0};
    char recv_info[QIANXUN_RECV_DATA_LEN] = {0};
    if (cmd_type == UNLOG_TYPE) {
        sprintf(cmd_temp, "%s %s\r\n", UNLOG_CMD, COMPORT1);
        rt = ntzx_qianxun_send(cmd_temp, strlen(cmd_temp));
        if (rt < 0) {
            return rt;
        }
    }  else if (cmd_type == GPGGA_TYPE) {
        sprintf(cmd_temp, "%s %f\r\n", GPGGA_CMD, upload_v);
        rt = ntzx_qianxun_send(cmd_temp, strlen(cmd_temp));
        if (rt < 0) {
            return rt;
        }
        rt = ntzx_qianxun_config_judge();
        if (rt < 0) {
            return rt;
        }
    } else if (cmd_type == GPHDT_TYPE) {
        sprintf(cmd_temp, "%s %f\r\n", GPHDT_CMD, upload_v);
        rt = ntzx_qianxun_send(cmd_temp, strlen(cmd_temp));
        if (rt < 0) {
            return rt;
        }
        rt = ntzx_qianxun_config_judge();
        if (rt < 0) {
            return rt;
        }
    } else if (cmd_type == GPYBM_TYPE) {
        sprintf(cmd_temp, "%s %f\r\n", GPYBM_CMD, upload_v);
        rt = ntzx_qianxun_send(cmd_temp, strlen(cmd_temp));
        if (rt < 0) {
            return rt;
        }
        rt = ntzx_qianxun_config_judge();
        printf("%d\n",rt);
        if (rt < 0) {
            return rt;
        }
    } else if (cmd_type == SAVE_TYPE) {
        sprintf(cmd_temp, "%s\r\n", SAVE_CMD);
        rt = ntzx_qianxun_send(cmd_temp, strlen(cmd_temp));
        if (rt < 0) {
            return rt;
        }
    }
    
    return NTZX_QIANXUN_SUCCESS;
}

/* 千寻惯导配置成功与否判断 */
static int ntzx_qianxun_config_judge(void)
{
    int i = 0;
    int rt;
    int time_out = 0;
    char recv_buf[QIANXUN_RECV_DATA_LEN] = {0};
    while (1) {
        rt = ntzx_qianxun_recv(recv_buf, sizeof(recv_buf));
        if (rt < 0) {
            time_out ++;
            usleep(1000);
            if (time_out > 40) {
                
                return NTZX_QIANXUN_CONF_ERR;
            }
        }
        // if (strstr(recv_buf, "$command") != NULL) {
        //     if (strstr(recv_buf, "response") != NULL) {
        //         if (strstr(recv_buf, "OK") != NULL) {
        //             return NTZX_QIANXUN_SUCCESS;
        //         }
                
        //         return NTZX_QIANXUN_CONF_ERR;
        //     }
        // }
        return NTZX_QIANXUN_SUCCESS;
        /* 如果接下来的40个数据内没有收到回复，则默认失败 */
        i++;
        if (i > 40) {
            
            return NTZX_QIANXUN_CONF_ERR;
        }
        continue;
    }
    return NTZX_QIANXUN_SUCCESS;
}

static int ntzx_qianxun_recv(char *recv_buf, int recv_len)
{
    int rt;
    int i;
    if (recv_buf == NULL || recv_len < 0) {
        return NTZX_QIANXUN_PARA_ERR;
    }
    for (i = 0; i < recv_len; i++) {
        rt = ntzx_usart_timeout_recv(g_ntzx_qianxun_ufd, &recv_buf[i], 1);
        if (rt <= 0) {
            printf("ntzx_usart_timeout_recv error\n");
            return NTZX_QIANXUN_RECV_ERR;
        }
        if (recv_buf[i] == '\n') {
            recv_buf[i + 1] = 0;
            return NTZX_QIANXUN_SUCCESS;
        }
    }
    return NTZX_QIANXUN_RECV_ERR;
}

static int ntzx_qianxun_send(char *recv_buf, int recv_len)
{
    return ntzx_usart_send(g_ntzx_qianxun_ufd, recv_buf, recv_len);
}

/* 千寻惯导串口初始化 */
static int ntzx_qianxun_usart_init(void)
{
    /* 初始化串口通信配置 */
    struct_usart_init_info qianxun_usart_info;
    qianxun_usart_info.baudrate = 115200; // 波特率
    qianxun_usart_info.databit = 8; // 传输字节长度
    qianxun_usart_info.fctl = 0; // 流控制  0: 无, 1: 硬件, 2: 软件
    qianxun_usart_info.parity = 0; // 校验位  0: 无, 1: 奇  , 2: 偶
    qianxun_usart_info.stopbit = 1; // 停止位  1或2
    //qianxun_usart_info.tty_dev_name = "/dev/ttyUSB0"; // 驱动加载的文件名“/dev/tty....”
    qianxun_usart_info.tty_dev_name = "/dev/ttyUSB0"; 
    g_ntzx_qianxun_ufd = ntzx_usart_init(&qianxun_usart_info); // 初始化串口通信
    if (g_ntzx_qianxun_ufd < 0) {
        return g_ntzx_qianxun_ufd;
    }
    return NTZX_QIANXUN_SUCCESS;
}

/* 千寻惯导初始化 */
int ntzx_qianxun_init(void)
{
    int rt;
    /* 获取串口文件名 */
    rt = ntzx_get_conf_key_string(QIANXUN_TITLE, QIANXUN_USART_KEY,
                                  g_qianxun_tty_dev_name,
                                  sizeof(g_qianxun_tty_dev_name) - 1);
    if (rt < 0) {
        return rt;
    }
    /* 千寻惯导串口初始化 */
    rt = ntzx_qianxun_usart_init();
    if (rt < 0) {
        return rt;
    }
    /* 将千寻惯导串口初始化成功写入日志 */
    ntzx_stereotypes_log_write_buf("qianxun usart init success!!!");
    /* 惯导配置 */
    /* 配置接收导航信息 */
    while (ntzx_qianxun_config(GPYBM_TYPE, 0.05f)) {
        sleep(1);
    }
    
    ntzx_stereotypes_log_write_buf("GPYBM init success!!!");
    // /* 配置接收航向角信息 */
    // while (ntzx_qianxun_config(GPHDT_TYPE, 0.05f)) {
    //     sleep(1);
    // }
    // ntzx_stereotypes_log_write_buf("GPHDT init success!!!");
    /* 配置接收惯导信息 */
    /* 该版本无惯导信息 */
    // while (ntzx_qianxun_config(GPTRA_TYPE, 0.05f)) {

    // }
    /* 保存配置信息 */
    rt = ntzx_qianxun_config(SAVE_TYPE, 0);
    if (rt < 0) {
        return rt;
    }
    return NTZX_QIANXUN_SUCCESS;
}

/* 重启串口设备 */
static void ntzx_qianxun_restart(void)
{
   
}

void qianxun_flush(void)
{
    tcflush(g_ntzx_qianxun_ufd, TCIFLUSH);
}