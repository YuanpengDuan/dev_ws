/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_conf_app.c
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：读取配置文件，更新配置文件
 * ******************************************************************/
#include "planning/ntzx_conf_app.hpp"
#include "planning/ntzx_file_drv.hpp"

#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <ctype.h>

#define TEMPLINE_LENMAX 1024
#define CONF_FILENAME  "config.ini"
#define CONF_FILENAME1 "config1.ini"

/* 读写配置文件的锁 */
static pthread_mutex_t g_mutex_conf_rw = PTHREAD_MUTEX_INITIALIZER;
/* 读取使用的临时行存储，暂使用1024表示行长度 */
static char g_templine[TEMPLINE_LENMAX];
/* 读取使用配置路径 */
static char g_config_path[NTZX_CONF_PATH_MAX];

/* 配置文件初始化函数 */
int ntzx_conf_app_init(void)
{
    /* 获取配置文件路径 */
    return ntzx_get_current_path(CONF_FILENAME, g_config_path, NTZX_CONF_PATH_MAX);
}

/* 从配置文件中读取数据 */
int ntzx_get_conf_key_string(const char *title, const char *key, char *value, int value_len)
{
    FILE *fp = NULL;
    char *temp = NULL;
    int i;
    int j = 0;
    if ((title == NULL) || (key == NULL) || (value == NULL) || (value_len <= 0)) {
        return NTZX_CONF_PARA_ERR;
    }
    pthread_mutex_lock(&g_mutex_conf_rw);
    memset(g_templine, 0, TEMPLINE_LENMAX);
    fp = fopen(g_config_path, "r");
    if(fp == NULL) {
        pthread_mutex_unlock(&g_mutex_conf_rw);
        return NTZX_CONF_OPEN_ERR; 
    }
    while(!feof(fp)) {
        temp = fgets(g_templine, TEMPLINE_LENMAX, fp);
        if (temp == NULL) {
            /* 如果是到文本末尾则跳出循环 */
            if (feof(fp)) {
                fclose(fp);
                pthread_mutex_unlock(&g_mutex_conf_rw);
                return NTZX_CONF_NO_TITLE;
            } else {
                fclose(fp);
                pthread_mutex_unlock(&g_mutex_conf_rw);
                return NTZX_CONF_READ_ERR;
            }
        }
        /*如果该行首字符即是#，说明该行的就是一个注释行,则继续读下一行*/
        if (g_templine[0] == '#') {
            continue;
        }
        /*如果没有读到标题则跳过，继续读取下一行*/
        if (strstr(g_templine, title) == NULL) {
            continue;
        }
        /*找到标题栏目之后，寻找标题栏目下对应的键，然后取出值*/
        while (!feof(fp)) {
            temp = fgets(g_templine, TEMPLINE_LENMAX, fp);
            if (temp == NULL) {
                /* 如果是到文本末尾则跳出循环 */
                if (feof(fp)) {
                    fclose(fp);
                    pthread_mutex_unlock(&g_mutex_conf_rw);
                    return NTZX_CONF_NO_KEY;
                } else {
                    fclose(fp);
                    pthread_mutex_unlock(&g_mutex_conf_rw);
                    return NTZX_CONF_READ_ERR;
                }
            }
            /* 剔除行buf中的空字符 */
            j = 0;
            for (i = 0; i < strlen(g_templine); i++) {
                /* 如果buf里面不是非空字符则保存下来 */
                if (0 == isspace(g_templine[i])) {
                    g_templine[j] = g_templine[i];
                    j++;
                }
            }
            g_templine[j] = 0;
            /*获取到了键值*/
            if ((strstr(g_templine, key) != NULL) && (strchr(g_templine, '=') != NULL)) {
                char *p; //创建一个临时变量
                p = strchr(g_templine, '=');
                p++;
                if (value_len < strlen(p)) {
                    fclose(fp);
                    pthread_mutex_unlock(&g_mutex_conf_rw);
                    return NTZX_CONF_PARA_ERR;
                }
                memcpy(value, p, strlen(p));
                fclose(fp);
                pthread_mutex_unlock(&g_mutex_conf_rw);
                return NTZX_CONF_SUCCESS;
            }
            /*已经到了下一个标题处，直接返回表示没找到想要的键值*/
            if ((g_templine[0] == '[') && (strchr(g_templine, ']') == NULL)) {
                fclose(fp);
                pthread_mutex_unlock(&g_mutex_conf_rw);
                return NTZX_CONF_NO_KEY;
            }
        }
    }
    fclose(fp);
    pthread_mutex_unlock(&g_mutex_conf_rw);
    return NTZX_CONF_NO_TITLE;
}

/* 改动配置文件(增加，修改，删除)，由于使用原因，配置文件基本只会修改，只实现修改功能 */
int ntzx_change_conf_ini(int action, const char *title, char *key, char *value)
{
    FILE *fp = NULL;
    FILE *fp1 = NULL;
    char *temp = NULL;
    if ((title == NULL) || (value == NULL) || (title == NULL)) {
        return NTZX_CONF_PARA_ERR;
    }
    pthread_mutex_lock(&g_mutex_conf_rw);
    memset(g_templine, 0, TEMPLINE_LENMAX);
    /* 打开配置文件 */
    fp = fopen(g_config_path, "r");
    if (fp == NULL) {
        pthread_mutex_unlock(&g_mutex_conf_rw);
        return NTZX_CONF_OPEN_ERR; 
    }
    /* 打开临时配置文件 */
    fp1 = fopen(CONF_FILENAME1, "w+");
    if (fp1 == NULL) {
        fclose(fp);
        pthread_mutex_unlock(&g_mutex_conf_rw);
        return NTZX_CONF_OPEN_ERR; 
    }
    while (!feof(fp)) {
        temp = fgets(g_templine, TEMPLINE_LENMAX, fp);
        if (temp == NULL) {
            /* 如果是到文本末尾则跳出循环 */
            if (feof(fp)) {
                fclose(fp);
                fclose(fp1);
                remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                pthread_mutex_unlock(&g_mutex_conf_rw);
                return NTZX_CONF_NO_TITLE;
            } else {
                fclose(fp);
                fclose(fp1);
                remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                pthread_mutex_unlock(&g_mutex_conf_rw);
                return NTZX_CONF_READ_ERR;
            }
        }
        if (fputs(g_templine, fp1) <= 0) {
            fclose(fp);
            fclose(fp1);
            remove(CONF_FILENAME1); // 未成功，删除临时配置文件
            pthread_mutex_unlock(&g_mutex_conf_rw);
            return NTZX_CONF_WRITE_ERR;
        }
        /* 如果不是当前标题栏则直接将 内容写入fp1 */
        if (NULL == strstr(g_templine, title)) {
            continue;
        }
        /* 如果是修改文件动作 */
        if (action == NTZX_CONF_ACTION_CHANGE) {
            while (!feof(fp)) {
                temp = fgets(g_templine, TEMPLINE_LENMAX, fp);
                if (temp == NULL) {
                    /* 如果是到文本末尾则跳出循环 */
                    if (feof(fp)) {
                        fclose(fp);
                        fclose(fp1);
                        remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                        pthread_mutex_unlock(&g_mutex_conf_rw);
                        return NTZX_CONF_NO_KEY;
                    } else {
                        fclose(fp);
                        fclose(fp1);
                        remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                        pthread_mutex_unlock(&g_mutex_conf_rw);
                        return NTZX_CONF_READ_ERR;
                    }
                }
                /* 获取到了键值 */
                if ((strstr(g_templine, key) != NULL) && (strchr(g_templine, '=') != NULL)) {
                    memset(g_templine, 0, TEMPLINE_LENMAX);
                    /* 写入新的键值对 */
                    sprintf(g_templine,"%s = %s", key, value);
                    if (fputs(g_templine, fp1) <= 0) {
                        fclose(fp);
                        fclose(fp1);
                        remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                        pthread_mutex_unlock(&g_mutex_conf_rw);
                        return NTZX_CONF_WRITE_ERR;
                    }
                    /* 将剩余内容写入 */
                    while (!feof(fp)) {
                        temp = fgets(g_templine, TEMPLINE_LENMAX, fp);
                        if (temp == NULL) {
                            /* 如果是到文本末尾则跳出循环 */
                            if (feof(fp)) {
                                break;
                            } else {
                                fclose(fp);
                                fclose(fp1);
                                remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                                pthread_mutex_unlock(&g_mutex_conf_rw);
                                return NTZX_CONF_READ_ERR;
                            }
                        }
                        if (fputs(g_templine, fp1) <= 0) {
                            fclose(fp);
                            fclose(fp1);
                            remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                            pthread_mutex_unlock(&g_mutex_conf_rw);
                            return NTZX_CONF_WRITE_ERR;
                        }
                    }
                    fclose(fp);
                    fclose(fp1);
                    remove(CONF_FILENAME);
                    rename(CONF_FILENAME1, CONF_FILENAME);
                    chmod(CONF_FILENAME, S_IRWXU | S_IRWXG | S_IRWXO);
                    pthread_mutex_unlock(&g_mutex_conf_rw);
                    return NTZX_CONF_SUCCESS;
                }
                /* 已经到了下一个标题处，直接返回表示没找到想要的键值 */
                if ((g_templine[0] == '[') && (strchr(g_templine, ']') == NULL)) {
                    fclose(fp);
                    fclose(fp1);
                    remove(CONF_FILENAME1); // 未成功，删除临时配置文件
                    pthread_mutex_unlock(&g_mutex_conf_rw);
                    return NTZX_CONF_NO_KEY;
                }
            }
        }
    }
    fclose(fp);
    fclose(fp1);
    remove(CONF_FILENAME1); // 未成功，删除临时配置文件
    pthread_mutex_unlock(&g_mutex_conf_rw);
    return NTZX_CONF_NO_TITLE;
}