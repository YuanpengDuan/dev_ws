/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_conf_app.h
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：读取配置文件，更新配置文件的头文件
 * ******************************************************************/

#ifndef _NTZX_CONF_APP_H_
#define _NTZX_CONF_APP_H_

#define NTZX_CONF_SUCCESS       0
#define NTZX_CONF_PARA_ERR      (-31)
#define NTZX_CONF_READ_ERR      (-32)
#define NTZX_CONF_WRITE_ERR     (-33)
#define NTZX_CONF_OPEN_ERR      (-34)
#define NTZX_CONF_NO_TITLE      (-35)
#define NTZX_CONF_NO_KEY        (-36)

#define NTZX_CONF_ACTION_ADD        1  // 增加配置文件
#define NTZX_CONF_ACTION_DELECT     2  // 删除部分配置文件键值
#define NTZX_CONF_ACTION_CHANGE     3  // 修改部分键值内容

#define NTZX_CONF_PATH_MAX      256

int ntzx_conf_app_init(void);
char *ntzx_get_ini_path(void);
int ntzx_get_conf_key_string(const char *title, const char *key, char *value, int value_len);
int ntzx_change_conf_ini(int action, const char *title, char *key, char *value);

#endif