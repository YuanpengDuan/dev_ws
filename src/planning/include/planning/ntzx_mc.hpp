/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_mc.h
 * 作者：任家豪
 * 日期：2021年2月26日
 * 描述：底层程序设置
 * ******************************************************************/

#ifndef _NTZX_MC_H_
#define _NTZX_MC_H_

#include "ntzx_usart_drv.hpp"

/* 定义各种错误返回值 */
#define NTZX_MC_SUCCESS 0
#define NTZX_MC_NO_SOCKET       (-101)
#define NTZX_MC_SEND_ERR        (-102)
#define NTZX_MC_RECV_INFO_ERR   (-103)
#define NTZX_MC_PARA_ERR        (-104)

#define  UART_MAX_USED  (2)
#define UART_PORT_ZERO  (0)       
#define UART_PORT_ONE   (0)



/* 关于参数action 各项值的定义,unsigned char action 的高3位没有用到，暂时保留 */
#define NTZX_MC_ACTION_NULL			     	      0x00	//0x00：表示无操作
#define	NTZX_MC_ACTION_STOP  			          0x01	//0x01：表示汽车停止，action的最低位
#define	NTZX_MC_ACTION_GO_FORWARD 	 0x02	//0x02：表示汽车前行，action的次低位
#define	NTZX_MC_ACTION_GO_BACKED	        0x04	//0x04： 表示汽车倒车
#define	NTZX_MC_ACTION_TURN_LEFT		     0x08	//0x08：  表示汽车左拐
#define NTZX_MC_ACTION_TURN_RIGHT	   0x10	//0x10：表示汽车右拐
#define	NTZX_MC_ACTION_TWIRL_LEFT		 0x20	//0x08：表示汽车原地左转
#define NTZX_MC_ACTION_TWIRL_RIGHT      0x40	//0x10：表示汽车原地右转

#define NTZX_MC_TURN_SPEED          700
#define NTZX_MC_TURN_DEGREE         65

/* 关于参数io_ctrl 各项值的定义,unsigned int io_ctrl第8~31位为预留位，暂时保留，可能会考虑是否开灯 */

#define NTZX_MC_IO_CMD_TRUMPET          0x00000001  //喇叭 , unsigned int io_ctrl的最低位
#define NTZX_MC_IO_CMD_HIGH_BEAM        0x00000002  //远光灯
#define NTZX_MC_IO_CMD_LOW_BEAM         0x00000004  //近光灯
#define NTZX_MC_IO_CMD_LEFT_TURNLAMP    0x00000008  //左转灯
#define NTZX_MC_IO_CMD_RIGHT_TURNLAMP   0x00000010  //右转灯
#define NTZX_MC_IO_CMD_BACKUP_LAMP      0x00000020  //倒车灯
#define NTZX_MC_IO_CMD_RUNNING_LAMP     0x00000040  //行驶灯
#define NTZX_MC_IO_CMD_ALARM_LAMP       0x00000080  //警示灯


/*规定最大速度和转角，向之后大概率还会调整*/
#define NTZX_MC_SPEED_MM_MAX            10000     //单位为mm/s
#define NTZX_MC_STEER_ANGLE             60

/*action_sta:车辆当前运动状态
io_sta：车辆当前IO口控制状态
speed_mm：当前车速，车速值显示为当前四个轮胎中速度的绝对值最大值。
steer_angle：当前转向角（即将四个轮胎中的转速差转换成转向角度）。
vol：当前小车的电池电压。
chassis_sta：当前车辆的底盘状态，只是根据已有资料中显示有底盘状态，因此提供了一个这个的参数。
*/

/*车辆反馈结构体*/
typedef struct mc_state_info{
    unsigned char action_sta;   // 运动状态
    unsigned int io_sta;        // io状态
    short speed_mm;             // 速度
    short steer_angle;          // 转向角
    unsigned short vol;         // 电压
    unsigned short chassis_sta; // 底盘状态
}struct_mc_sta;

/*小车控制结构体及内部参数*/
typedef struct mc_ctrl_info{
    unsigned char action;
    unsigned int io_ctrl;
    short speed_mm;   //毫米每秒
    short steer_angle;

}struct_mc_ctrl;

int ntzx_mc_usart_init(void);
void *ntzx_mc_main(void *arg);
int ntzx_mc_ctrl();
int ntzx_get_mc_state(struct_mc_sta *mc_state);
#endif
