/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_file_drv.c
 * 作者：任家豪
 * 日期：2021年2月2日
 * 描述：文件的打开与读写功能
 * ******************************************************************/
#include "planning/ntzx_file_drv.hpp"

#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

/*初始化程序*/
int ntzx_file_init(const char* path, FILE **file, const char *mode)
{
    if (path == NULL) {
        return NTZX_FILE_PARA_ERR;
    }
    *file = fopen(path, mode);
    if (*file == NULL) {
        return NTZX_FILE_OPEN_ERR;
    }
    return NTZX_FILE_SUCCESS;
}

/* 获取当前程序下目录下文件的绝对路径 */
int ntzx_get_current_path(const char *pFileName, char *path, int path_len)
{
    char pidfile[64];
    int bytes;
    int fd;
    char * p = NULL;

    if ((pFileName == NULL) || (path == NULL) || (path_len <= 0)) {
        return NTZX_FILE_PARA_ERR;
    }
    sprintf(pidfile, "/proc/%d/cmdline", getpid());
    fd = open(pidfile, O_RDONLY, 0);
    bytes = read(fd, path, path_len);
    if (bytes <= 0) {
        return NTZX_FILE_READ_ERR;
    }
    close(fd);
    path[path_len - 1] = '\0';
    p = &path[strlen(path)];
    /*确保在最后一个'/'后面没有字符*/
    do {
        *p = '\0';
        p--;
    } while (*p != '/');
    p++;
    // 从最后一个'/'字符后面开始写入文件名
    memcpy(p, pFileName, strlen(pFileName));
    return NTZX_FILE_SUCCESS;
}

/* 写入字符串 */
int ntzx_file_write_string(FILE *file, const char* data, int date_len)
{
    if ((file == NULL)  || (data == NULL) || (date_len <= 0)) {
        return NTZX_FILE_PARA_ERR;
    }
    fprintf(file, "%s", data);
    fflush(file);
    return NTZX_FILE_SUCCESS;
}

/* 写入一个字符 */
int ntzx_file_write_char(FILE *file, char data)
{
    if ((file == NULL) || (data > 0)) {
        return NTZX_FILE_PARA_ERR;
    }
    fprintf(file, "%c", data);
    fflush(file);
    return NTZX_FILE_SUCCESS;
}