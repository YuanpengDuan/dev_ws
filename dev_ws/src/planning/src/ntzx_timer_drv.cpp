/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_timer_drv.c
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：设置时间，获取时间到达标志
 * ******************************************************************/
#include "planning/ntzx_timer_drv.hpp"

#include <stdio.h>      //NULL
 
 //timer->start_cn保存当前时刻；timer->alarm_time保存定时时间：单位微妙，us
void Timer_Set(ntzx_systimer *timer, unsigned long alarm_time)
{
    gettimeofday(&timer->start_cnt, NULL);
    timer->alarm_time = alarm_time;    
}

//通过当前时刻和timer->start_cnt作差判断是否定时时间到
int Timer_GetReached(ntzx_systimer *timer)
{
    if(timer->alarm_time >= 0)//定时时间要大于0
    {
        //  printf("2\n");
        struct timeval end_cnt;
        gettimeofday(&end_cnt, NULL);
        unsigned long time_diff = (end_cnt.tv_sec  - timer->start_cnt.tv_sec) * 1000000 + (end_cnt.tv_usec - timer->start_cnt.tv_usec);
        printf("time_diff = %d,alarm_time = %d\n",time_diff,timer->alarm_time);
        if( time_diff  > (timer->alarm_time) ) {
            return NTZX_TIMER_TRIGGER;
        }
        else{
            return NTZX_TIMER_NO_TRIGGER;
        }    
    }
    return NTZX_TIMER_PARAM_ERR;
}

void get_abstime_wait(int microseconds, struct timespec *abstime)
{
    struct timeval tv;
    long long absmsec;
    gettimeofday(&tv, NULL);
    absmsec = tv.tv_sec * 1000ll + tv.tv_usec / 1000ll;
    absmsec += microseconds;

    abstime->tv_sec = absmsec / 1000ll;
    abstime->tv_nsec = absmsec % 1000ll * 1000000ll;
}