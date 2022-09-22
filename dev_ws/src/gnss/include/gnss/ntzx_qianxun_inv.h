/********************************************************************
    * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
    * 文件名：ntzx_qianxun_inv.h
    * 作者：任家豪
    * 日期：2021年2月22日
    * 描述：千寻惯导的初始化，以及一些对外封装
 * ******************************************************************/

#ifndef _NTZX_QIANXUN_INV_H_
#define _NTZX_QIANXUN_INV_H_

#define NTZX_QIANXUN_SUCCESS        0
#define NTZX_QIANXUN_PARA_ERR       (-61)
#define NTZX_QIANXUN_CONF_ERR       (-62)
#define NTZX_QIANXUN_DELAY_SYNC     (-63)
#define NTZX_QIANXUN_ANALYSIS_ERR   (-64)
#define NTZX_QIANXUN_STATE_ERR      (-65)
#define NTZX_QIANXUN_RECV_ERR       (-66)
#define NTZX_QIANXUN_DATA_INVALID   (-66)
#define NTZX_QIANXUN_DATA_ERR       (-67)

#define NTZX_QIANXUN_QUAL_NONE      0
#define NTZX_QIANXUN_QUAL_SINGLE    1
#define NTZX_QIANXUN_QUAL_SBAS      2
#define NTZX_QIANXUN_QUAL_RTKFIX    4
#define NTZX_QIANXUN_QUAL_RTKFLOAT  5
#define NTZX_QIANXUN_QUAL_INS       6
#define NTZX_QIANXUN_QUAL_USERS     7

#pragma pack(push, 1)
typedef struct gps_time {
    int hour;  // 当条gps数据的小时
    int min;   // 当条gps数据的分钟
    int s;     // 当条gps数据的秒数
    int ms;    // 当条gps数据的毫秒
} str_gps_time;

typedef struct gps_data {
    int frame;
    str_gps_time time;      // 当条数据的北京时间
    double lat;             // 纬度
    unsigned char lat_dir;   // 北纬，还是南纬
    double lon;              // 经度
    unsigned char lon_dir;   // 西经，东经
    unsigned char qual;      // GPS质量指标
    double courseAngle;      // 航向角度值
    double Picth;            //俯仰角
    unsigned char u_unit;    // 地理高度单位m
} str_gps_data;

typedef struct vel_date_data{
    float Vel_N;
    float Vel_E;
    float Vel_D;
    float Vel_G;

}str_vel_data;


typedef struct inv_to_pl {
    int frame;
    double lat;             // 纬度
    double lon;             // 经度
    double courseAngle; // 航向角度值
    double speedmm;//后加的
} str_inv_to_pl;
#pragma pack(pop)

void *ntzx_qianxun_inv_main(void *arg); // 线程主函数
int ntzx_get_inv_to_pl(str_inv_to_pl *nav_info); // 提供给PL的导航信息
int ntzx_qianxun_init(void);
void qianxun_flush(void);   //刷新千寻接口缓冲
#endif

