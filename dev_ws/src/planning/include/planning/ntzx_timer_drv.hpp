/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_timer_drv.h
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：设置时间，获取时间到达标志的接口
 * ******************************************************************/

#ifndef _NTZX_TIMER_DRV_H_
#define _NTZX_TIMER_DRV_H_

#include <sys/time.h>

#define NTZX_TIMER_TRIGGER      0
#define NTZX_TIMER_NO_TRIGGER   (-1)
#define NTZX_TIMER_PARAM_ERR   (-2)

typedef struct typeSYSTIMER
{
    struct timeval start_cnt;
    unsigned long alarm_time;
}ntzx_systimer;

void Timer_Set(ntzx_systimer *timer, unsigned long alarm_time); // 设置报警周期

void get_abstime_wait(int microseconds, struct timespec *abstime); 

int Timer_GetReached(ntzx_systimer *timer); // 查询报警周期是否达到


#endif