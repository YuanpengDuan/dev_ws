/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_usart_drv.h
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：底层的串口通信调用接口，以及设置参数的结构体定义
 * ******************************************************************/

#ifndef _NTZX_USART_DRV_H_
#define _NTZX_USART_DRV_H_

#define NTZX_USART_SUCCESS       0    // 串口成功
#define NTZX_USART_PARA_ERR      (-21) // 传入参数错误
#define NTZX_USART_OPEN_ERR      (-22) // 打开文件失败
#define NTZX_USART_CONF_ERR      (-23) // 配置属性失败
#define NTZX_USART_READ_ERR      (-24) // 读取内容失败
#define NTZX_USART_WRITE_ERR     (-25) // 写入内容失败
#define NTZX_USART_RECV_OVERTIME (-26) // 接收超时

typedef struct usart_init_info{
    int  baudrate;  // 波特率
    char databit;   // 传输字节长度
    char fctl;      // 流控制  0: 无, 1: 硬件, 2: 软件
    char parity;    // 校验位  0: 无, 1: 奇  , 2: 偶
    char stopbit;   // 停止位  1或2
    const char *tty_dev_name; // 驱动加载的文件名“/dev/tty....”
} struct_usart_init_info;

int ntzx_usart_init(struct_usart_init_info *usart_info);     // 串口初始化
void ntzx_usart_close(int usart_fd);                         // 关闭描述符
int ntzx_usart_send(int usart_fd, const char *data, int datalen);  // 串口发送
int ntzx_usart_recv(int usart_fd, char *data, int datalen);  // 串口写入
int ntzx_usart_timeout_recv(int usart_fd, char *data, int datalen); // 带超时的串口写入

#endif