/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_log_app.c
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：日志打印功能接口
 * ******************************************************************/

#ifndef _NTZX_LOG_APP_H_
#define _NTZX_LOG_APP_H_

#define NTZX_LOG_SUCCESS        0
#define NTZX_LOG_OPEN_ERR      (-41)
#define NTZX_LOG_WRITE_ERR     (-42)
#define NTZX_LOG_PARA_ERR      (-43)

int ntzx_log_app_init(void);
int ntzx_log_write(const char *value, int value_len);
void ntzx_close_logfile(void);
int ntzx_stereotypes_log_write_err(char *title, int error_num);
int ntzx_stereotypes_log_write_buf(char *data);

#endif