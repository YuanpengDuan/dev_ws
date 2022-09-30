/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_usart_drv.c
 * 作者：任家豪
 * 日期：2021年2月1日
 * 描述：底层的串口通信调用，包括初始化，接收与发送
 * ******************************************************************/
#include "planning/ntzx_usart_drv.hpp"

#include <stdio.h>              // printf   
#include <fcntl.h>              // open   
#include <string.h>             // bzero   
#include <stdlib.h>             // exit   
#include <sys/times.h>          // times   
#include <sys/types.h>          // pid_t   
#include <termios.h>            // termios, tcgetattr(), tcsetattr()   
#include <unistd.h>   
#include <sys/ioctl.h>          // ioctl   
#include <sys/select.h>

#define TIMEOUT_SEC(buflen,baud) ((buflen)*20/(baud)+2)  //接收超时
#define TIMEOUT_USEC 0

static int ntzx_swicth_baudrate(int baudrate)
{
    switch(baudrate) {
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        default:
            return B9600;
    }
}

static void ntzx_swicth_fctl(struct termios *termios_p, int fctl)
{
    switch (fctl) {
        case 0:
            termios_p->c_cflag &= ~CRTSCTS;         // 没有控制
            break;
        case 1:
            termios_p->c_cflag |= CRTSCTS;       // 硬件控制
            break;
        case 2:
            termios_p->c_iflag |= IXON | IXOFF |IXANY; // 软件控制
            break;
    }
}

static void ntzx_switch_databits(struct termios *termios_p, int databit)
{
    switch (databit) {  
        case 5:  
            termios_p->c_cflag |= CS5;
            break;
        case 6:  
            termios_p->c_cflag |= CS6;
            break;
        case 7:  
            termios_p->c_cflag |= CS7;
            break;
        default:  
            termios_p->c_cflag |= CS8;  
    }  
}

static void ntzx_switch_parity(struct termios *termios_p, int parity)
{
    switch(parity){  
        case 0:  
            termios_p->c_cflag &= ~PARENB;     //无奇偶
            break;  
        case 1:
            termios_p->c_cflag |= PARENB;      //奇
            termios_p->c_cflag &= ~PARODD;  
            break;  
        case 2: 
            termios_p->c_cflag |= PARENB;      //偶  
            termios_p->c_cflag |= PARODD;  
            break;  
    }  
}

static void ntzx_switch_stopbit(struct termios *termios_p, int stopbit)
{
    if(stopbit == 2){  
        termios_p->c_cflag |= CSTOPB;  //2 stop bits   
    }  
    else{  
        termios_p->c_cflag &= ~CSTOPB; //1 stop bits   
    }
}

int ntzx_usart_init(struct_usart_init_info *usart_info)
{
    /* 定义一个文件描述符 */
    int usart_fd = -1;
    /* 定一个老的串口配置，和一个新的串口配置参数 */
    struct termios termios_old, termios_new;
    /* 检验传入参数 */
    if ((usart_info == NULL) || (usart_info->tty_dev_name == NULL)) {
        return NTZX_USART_PARA_ERR;
    }
    /* 将两个变量数据清0 */
    bzero(&termios_old, sizeof(termios_old));
    bzero(&termios_new, sizeof(termios_new));
    cfmakeraw(&termios_new); /* 初始化成默认值 */
    
    /* 打开文件串口文件描述符 */
    usart_fd = open(usart_info->tty_dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (usart_fd < 0) {
        return NTZX_USART_OPEN_ERR;
    }
    /* 转换波特率为系统相关定义值 */
    usart_info->baudrate = ntzx_swicth_baudrate(usart_info->baudrate);
    
    cfsetispeed(&termios_new, usart_info->baudrate);    /* 填入串口输入端的波特率 */  
    cfsetospeed(&termios_new, usart_info->baudrate);    /* 填入串口输出端的波特率 */
    termios_new.c_cflag |= CLOCAL;                             /* 控制模式，保证程序不会成为端口的占有者 */  
    termios_new.c_cflag |= CREAD;                              /* 控制模式，使能端口读取输入的数据 */
    ntzx_swicth_fctl(&termios_new, usart_info->fctl);        /* 控制模式映射 */
    termios_new.c_cflag &= ~CSIZE;                             /* 控制模式，屏蔽字符大小位 */
    ntzx_switch_databits(&termios_new, usart_info->databit); /* 控制模式，单字节长度设置 */
    ntzx_switch_parity(&termios_new, usart_info->parity);    /* 控制模式，奇偶性设置 */
    ntzx_switch_stopbit(&termios_new, usart_info->stopbit);  /* 控制模式，停止位设置 */
    termios_new.c_oflag &= ~OPOST;           /* 输出模式，原始数据输出 */
    termios_new.c_cc[VMIN]  = 1;            /* 控制字符, 所要读取字符的最小数量 */
    termios_new.c_cc[VTIME] = 1;            /* 控制字符, 读取第一个字符的等待时间    unit: (1/10)second */
    
    tcflush(usart_fd, TCIFLUSH);               /* 溢出的数据可以接收，但不读 */

    if(-1 == tcsetattr(usart_fd, TCSANOW, &termios_new)) { /* 设置新属性，TCSANOW：所有改变立即生效 */
        ntzx_usart_close(usart_fd);      /* 关闭文件描述符 */
        return NTZX_USART_CONF_ERR;
    }
    return usart_fd;
}

/* 关闭串口文件描述符 */
void ntzx_usart_close(int usart_fd)
{  
    close(usart_fd);  
}

/* 发送成功返回发送成功字数，失败返回对应失败类型 */
int ntzx_usart_send(int usart_fd, const char *data, int datalen)  
{  
    int len = 0;
    if (data == NULL) {
        return NTZX_USART_PARA_ERR;
    }
    len = write(usart_fd, data, datalen);
    if (len == datalen) {
        return len;
    } else { 
        tcflush(usart_fd, TCOFLUSH);
        return NTZX_USART_WRITE_ERR;
    }
}

/* 如果读取成功返回串口ID，如果失败返回对应失败类型 */
int ntzx_usart_recv(int usart_fd, char *data, int datalen)
{ 
    int i;
    int readlen;  
    fd_set  fs_read;
    
    if ((usart_fd <= 0) || (data == NULL) || (datalen <= 0)) {
        return NTZX_USART_PARA_ERR;
    }
    FD_ZERO(&fs_read);
    FD_SET(usart_fd, &fs_read);
    if (select(usart_fd + 1, &fs_read, NULL, NULL, NULL) > 0) {
        if (FD_ISSET(usart_fd, &fs_read)) {
            readlen = read(usart_fd, data, datalen);
            return readlen;
        }
    }
    return NTZX_USART_RECV_OVERTIME;
}

/* 如果读取成功返回串口ID，如果失败返回对应失败类型,带超时 */
int ntzx_usart_timeout_recv(int usart_fd, char *data, int datalen)
{ 
    int i;
    int readlen;  
    fd_set  fs_read;  
    struct timeval tv_timeout;  
    
    if ((usart_fd <= 0) || (data == NULL) || (datalen <= 0)) {
        return NTZX_USART_PARA_ERR;
    }
    FD_ZERO(&fs_read);
    FD_SET(usart_fd, &fs_read);
    tv_timeout.tv_sec = TIMEOUT_SEC(datalen, 115200); /* 直接使用115200 */
    tv_timeout.tv_usec = TIMEOUT_USEC;
    if (select(usart_fd + 1, &fs_read, NULL, NULL, &tv_timeout) > 0) {
        if (FD_ISSET(usart_fd, &fs_read)) {
            readlen = read(usart_fd, data, datalen);
            return readlen;
        }
    }
    return NTZX_USART_RECV_OVERTIME;
}