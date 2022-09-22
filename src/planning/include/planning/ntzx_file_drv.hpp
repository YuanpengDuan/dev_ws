/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_file_drv.h
 * 作者：任家豪
 * 日期：2021年2月2日
 * 描述：文件的打开与读写功能接口
 * ******************************************************************/

#ifndef _NTZX_FILE_DRV_H_
#define _NTZX_FILE_DRV_H_

#include <stdio.h>

#define NTZX_FILE_SUCCESS            0    // 文件成功
#define NTZX_FILE_PARA_ERR           (-11) // 传入参数错误
#define NTZX_FILE_OPEN_ERR           (-12) // 打开文件失败
#define NTZX_FILE_READ_ERR           (-13) // 读取文件失败

int ntzx_file_init(const char* path, FILE **file, const char *mode);
int ntzx_file_write_string(FILE *file, const char* data, int date_len); // 写入一个字符串
int ntzx_file_write_char(FILE *file, char data); // 写入一个字符
int ntzx_get_current_path(const char *pFileName, char *path, int path_len); // 获取当前程序下目录下文件的绝对路径

#endif