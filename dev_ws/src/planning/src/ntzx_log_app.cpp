/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_log_app.c
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：日志打印功能
 * ******************************************************************/
#include "planning/ntzx_log_app.hpp"
#include "planning/ntzx_conf_app.hpp"
#include "planning/ntzx_file_drv.hpp"

#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <pthread.h>

// 配置文件的标题
#define LOG_TITLE "LOG" 
// 配置文件的键
#define LOG_MODE_KEY "LOG_MODE"
#define LOG_PATH_KEY "LOG_PATH"

/* LOG_MODE 的值 */
#define LOG_CLOSE 0
#define LOG_OPEN  1
#define LOG_PRINT 2

static pthread_mutex_t g_mutex_log_app_write = PTHREAD_MUTEX_INITIALIZER;

FILE *g_log_file = NULL;
int g_log_mode = -1;

/* 日志文件初始化 */
int ntzx_log_app_init(void)
{
    int rt;
    char value[256] = {0}; // 给value这个接收变量足够大的空间，256.
    rt = ntzx_get_conf_key_string(LOG_TITLE, LOG_MODE_KEY, value, sizeof(value));
    if (rt < 0) {
        return rt;
    }
    g_log_mode = atoi(value);
    rt = ntzx_get_conf_key_string(LOG_TITLE, LOG_PATH_KEY, value, sizeof(value));
    if (rt < 0) {
        return rt;
    }
    struct timeval tv;
    struct tm* p;
    gettimeofday(&tv, NULL);
    p = localtime((const time_t *)&tv.tv_sec);
    sprintf(value, "%s-%04d-%02d-%02d-%02d:%02d:%02d",  value,
             1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday,
             p->tm_hour, p->tm_min, p->tm_sec);
    printf("%s",value);
    g_log_file = fopen(value, "w+");
    if (g_log_file == NULL) {
        return NTZX_LOG_OPEN_ERR;
    }
    return NTZX_LOG_SUCCESS;
}

int ntzx_log_write(const char *value, int value_len)
{
    if (value == NULL && value_len <= 0) {
        return NTZX_LOG_PARA_ERR;
    }
    if (g_log_mode == LOG_CLOSE) {
        return NTZX_LOG_SUCCESS;
    }
    if (g_log_mode == LOG_PRINT) {
        printf("%s", value);
        return NTZX_LOG_SUCCESS;
    }
    if (g_log_mode == LOG_OPEN) {
        int rt = -1;
        pthread_mutex_lock(&g_mutex_log_app_write);
        rt = ntzx_file_write_string(g_log_file, value, value_len);
        pthread_mutex_unlock(&g_mutex_log_app_write);
        return rt;
    }
}

int ntzx_stereotypes_log_write_err(char *title, int error_num)
{
    char err_info[256] = {0};
    struct timeval tv;
    struct tm* p;
    gettimeofday(&tv, NULL);
    p = localtime((const time_t *)&tv.tv_sec);
    sprintf(err_info, "%04d.%02d.%02d.%02d:%02d:%02d.%03ld   %s:%d\n",
              1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday,
		      p->tm_hour, p->tm_min, p->tm_sec,
		      tv.tv_usec / 1000, title, error_num);
    return ntzx_log_write(err_info, sizeof(err_info));
}

int ntzx_stereotypes_log_write_buf(char *data)
{
    char data_info[1024] = {0};
    struct timeval tv;
    struct tm* p;
    gettimeofday(&tv, NULL);
    p = localtime((const time_t *)&tv.tv_sec);
    sprintf(data_info, "%04d.%02d.%02d.%02d:%02d:%02d.%03ld   %s\n",
              1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday,
		      p->tm_hour, p->tm_min, p->tm_sec,
		      tv.tv_usec / 1000, data);
    return ntzx_log_write(data_info, sizeof(data_info));
}

void ntzx_close_logfile(void)
{
    fclose(g_log_file);
}