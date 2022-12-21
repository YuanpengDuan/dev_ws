#include "planning/ntzx_mc.hpp"
#include "planning/ntzx_usart_drv.hpp"
#include "planning/ntzx_log_app.hpp"
#include "planning/ntzx_conf_app.hpp"
#include "planning/ntzx_timer_drv.hpp"
#include "planning/ntzx_planning_lane.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>                      // open   
#include <string.h>                  // bzero   
#include <sys/times.h>          // times   
#include <sys/types.h>          // pid_t   
#include <termios.h>              // termios, tcgetattr(), tcsetattr()  
#include <unistd.h>   
#include <sys/ioctl.h>             // ioctl
#include <pthread.h>

#define   CAR_LEN       (620)    //车子的宽度，单位mm
#define   RAD2DEGREE    (57.3)   //弧度转换成角度的系数
#define   ROWLENGTH     (64)	 //定义接收原始数据长度
#define   SPEEDMAX       1000   // 定义一个最大值
#define   MC_RECV_LEN   64
#define   MC_RECV_STA_PERIOD    50000

#define   MC_USART  "USART"
#define   MC_USART_KEY "USART_MC"

struct_mc_ctrl g_mc = {0};
static char g_mc_tty_dev_name[20] = {0};
//控制指令串，默认为停止状态
const unsigned char g_mc_stop_string[24] = { 237 ,222, 23 ,4, 2, 128, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 110, 4, 13, 10 };
static int g_usart_fd = -1;

/* 汽车当前状态值 */
static struct_mc_sta g_mc_state = {0};
static pthread_mutex_t g_mutex_mc_state_info = PTHREAD_MUTEX_INITIALIZER;

static int ntzx_mc_ctrl_to_string(struct_mc_ctrl ctrl ,unsigned char * str_cmd);
static void ntzx_mc_speed_check(struct_mc_ctrl * ctrl);
static void ntzx_mc_speed_write(unsigned char * str_cmd, short left, short right);
static int ntzx_mc_recv_info(struct_mc_sta *mc_sta);
static int ntzx_recv_data_handle(unsigned char* row, struct_mc_sta  *sta);

struct_mc_ctrl mc_ctrl;

void *ntzx_mc_main(void *arg)
{
    int rt;
    printf("start!\n");
    ntzx_systimer mc_sys = {0};
    /* 初始化MC */
    ntzx_mc_usart_init();
    if(rt == 0)
    {
        printf("ntzx_mc_usart_init success\n");
    }
    while(1){
        usleep(100000);
    ntzx_mc_ctrl();
    // printf("ctrl send success");
   
    // g_mc.action= NTZX_MC_ACTION_GO_FORWARD;
    //  g_mc.speed_mm =80;
    // g_mc.io_ctrl = 0;
    // g_mc.steer_angle = 0;
    // ntzx_mc_ctrl(g_mc);
    }
    // Timer_Set(&mc_sys, 100);
    // while (g_usart_fd < 0);
    // while (1) {
    //     if (Timer_GetReached(&mc_sys)) {
    //         continue;
    //     }
    //     Timer_Set(&mc_sys, MC_RECV_STA_PERIOD);
    //     pthread_mutex_lock(&g_mutex_mc_state_info);
    //     rt = ntzx_mc_recv_info(&g_mc_state);
    //     pthread_mutex_unlock(&g_mutex_mc_state_info);
    //     if (rt < 0) {
    //         ntzx_stereotypes_log_write_err("MC_MAIN:", rt);
    //     }
    // }
}

int ntzx_mc_ctrl()
{
   int rt = -1;

    unsigned char g_mc_ctrl_string[24] = { 237 ,222, 23 ,4, 2, 128, 6,80,1, 80, 1, 80, 1, 80, 1, 0, 0, 0, 0, 178, 3, 13, 10 };//mc底层格式
    //unsigned char g_mc_ctrl_string[24] = { "ED DE 17 04 02 80 06 50 01 50 01 50 01 50 01 00 00 00 00 B2 03 0D 0A "};
    if (g_usart_fd < 0) {
        return NTZX_MC_NO_SOCKET;
    }
    rt = ntzx_mc_ctrl_to_string(mc_ctrl, g_mc_ctrl_string);//将指令转换成mc底层格式 原本为ctrl
    if (rt < 0) {
        return rt;
    }

   ntzx_usart_send(g_usart_fd,(char*)g_mc_ctrl_string, 24);
//    printf("%d\n",g_mc_ctrl_string);
    // if (rt < 0) {
    //     return NTZX_MC_SEND_ERR;
    // }
    
    return NTZX_MC_SUCCESS;
}

static int ntzx_mc_ctrl_to_string(struct_mc_ctrl ctrl ,unsigned char * str_cmd)
{
    /* 对速度进行检测 */
//    ntzx_mc_speed_check(&ctrl); 
    switch (ctrl.action) {
        case NTZX_MC_ACTION_NULL: {

        }
        case NTZX_MC_ACTION_STOP: {
            for (int i = 0;i < 24; i++) {
                str_cmd[i] = g_mc_stop_string[i];
            }
            break;
        }
        case NTZX_MC_ACTION_GO_FORWARD: {
            double steer_angle_per;
            short speed_temp;
            steer_angle_per = ((double)ctrl.steer_angle) / 90;
            /* 向左拐 */
            if (steer_angle_per < 0) {
                steer_angle_per = -steer_angle_per;
                steer_angle_per = 1 - steer_angle_per;
                speed_temp = (short)(ctrl.speed_mm * steer_angle_per);
                /* 调用赋值各轮胎速度 */
                ntzx_mc_speed_write(str_cmd, speed_temp, ctrl.speed_mm);
            /* 向右拐 */
            } else if (steer_angle_per > 0) {
                steer_angle_per = 1 - steer_angle_per;
                speed_temp = (short)(ctrl.speed_mm * steer_angle_per);
                /* 调用赋值各轮胎速度 */
                ntzx_mc_speed_write(str_cmd, ctrl.speed_mm, speed_temp);
            } else {
                /* 调用赋值各轮胎速度 */
               ntzx_mc_speed_write(str_cmd, ctrl.speed_mm, ctrl.speed_mm);
           }
           break;
       }
        case NTZX_MC_ACTION_GO_BACKED: {
            
            break;       
        }

   }

    return NTZX_MC_SUCCESS;
}

int ntzx_mc_usart_init(void)
{
    struct_usart_init_info mc_usart_info;
    int rt = -1;
    /* 获取串口文件名 */
    // rt = ntzx_get_conf_key_string(MC_USART, MC_USART_KEY, g_mc_tty_dev_name,sizeof(g_mc_tty_dev_name) - 1);
    // if (rt < 0) {
    //     return rt;
    // }
    mc_usart_info.baudrate = 115200;
    mc_usart_info.databit = 8;
    mc_usart_info.fctl = 0;
    mc_usart_info.parity = 0;
    mc_usart_info.stopbit = 1;
    // mc_usart_info.tty_dev_name = "/dev/ttyUSB0";
    mc_usart_info.tty_dev_name = "/dev/pts/5";
    g_usart_fd = ntzx_usart_init(&mc_usart_info);
    if (g_usart_fd < 0) {
        return rt;
    }
    return NTZX_MC_SUCCESS;
}

int ntzx_get_mc_state(struct_mc_sta *mc_state)
{
    if (mc_state == NULL) {
        return NTZX_MC_PARA_ERR;
    }
    //pthread_mutex_lock(&g_mutex_mc_state_intzx_get_conf_key_string)
   
   if(g_mc_state.speed_mm<0)
        g_mc_state.speed_mm = 0;
    if (g_mc_state.speed_mm >SPEEDMAX)
    {
        g_mc_state.speed_mm = SPEEDMAX; /* code */
    }
    // if(ctrl->speed_mm<0)
    //     ctrl->speed_mm = 0;
    // if (ctrl->speed_mm >SPEEDMAX)
    // {
    //     ctrl->speed_mm = SPEEDMAX; /* code */
    // }
}

static void ntzx_mc_speed_write(unsigned char * str_cmd, short left, short right)
{
    //给各个轮子赋值
    unsigned char left_high = left / 255;
    unsigned char left_low = left % 255;
    unsigned char right_high = right / 255;
    unsigned char right_low = right % 255;
    //前右
    str_cmd[7] = right_low;
    str_cmd[8] = right_high;
    //前左
    str_cmd[9] = left_low;
    str_cmd[10] = left_high;
    //后右
    str_cmd[11] = right_low;
    str_cmd[12] = right_high;
    //后左
    str_cmd[13] = left_low;
    str_cmd[14] = left_high;
    //求校验和
    int sum = 622 + (left_high + left_low) * 2 + (right_high + right_low) * 2;
    unsigned char test_high = sum / 256;
    unsigned char test_low = sum % 256;

    str_cmd[19] = test_low;
    str_cmd[20] = test_high;
}

static int ntzx_mc_recv_info(struct_mc_sta *mc_sta)
{
    int i;
    int rt = -1;
    unsigned char string_recv[MC_RECV_LEN] = {0};
    unsigned char *p = &string_recv[0];
    for (i = 0; i < MC_RECV_LEN; i++) {
        ntzx_usart_recv(g_usart_fd, (char*)p ,1);
        p++;
    }
    /* 提取信息 */
    rt = ntzx_recv_data_handle(string_recv, mc_sta);
    if (rt < 0) {
        return rt;
    }
    return NTZX_MC_SUCCESS;
}

static int ntzx_recv_data_handle(unsigned char* row, struct_mc_sta  *sta)
{
    int i = 0;
    for ( i = 0; i < 64; i++)
    {
        if (row[i] == 237 && row[i+1] == 222)      //报文以237 、222为开头（ED DE）
        {   
            if (i > 39){ //小车上传报文总长为25，若检查到39位也没找到开头，则判定接受数据失败
                return NTZX_MC_RECV_INFO_ERR;		
            }
                
            if (row[i + 23] == 13 && row[i + 24] == 10)   //报文以13 、10为结尾（0D 0A）
            {   
                //到此说明找到报文的开头部分
                //0-3对应四个轮子的速度
                int Right_wheel_speed = row[i + 8]  * 256 + row[i + 7];
                //判断是否为负数
                if (row[i + 8]>=128) {
                        Right_wheel_speed= Right_wheel_speed-65536;
                }
                int Left_wheel_speed = row[i + 10] * 256 + row[i + 9];
                if (row[i + 10] >= 128) {
                    Left_wheel_speed = Left_wheel_speed - 65536;
                }

                if (Right_wheel_speed == 0 && Left_wheel_speed == 0)
                {
                    sta->action_sta = NTZX_MC_ACTION_STOP;
                    sta->speed_mm=0;
                    sta->steer_angle = 0;
                }
                else if (abs(Right_wheel_speed-Left_wheel_speed)<=10 && Right_wheel_speed >0)
                {
                    sta->action_sta = NTZX_MC_ACTION_GO_FORWARD;
                    sta->speed_mm = Right_wheel_speed;
                    sta->steer_angle = 0;
                }
                else if (abs(Right_wheel_speed-Left_wheel_speed)<=10 && Right_wheel_speed <0)
                {
                    sta->action_sta = NTZX_MC_ACTION_GO_BACKED;
                    sta->speed_mm = Right_wheel_speed;
                    sta->steer_angle = 0;
                }
                else if ((Right_wheel_speed-Left_wheel_speed)>10 && Left_wheel_speed>=0)
                {
                    sta->action_sta = NTZX_MC_ACTION_TURN_LEFT;
                    sta->speed_mm = Right_wheel_speed;
                    float rad = ((float) (Right_wheel_speed-Left_wheel_speed))/CAR_LEN;
                    int  turn_degree = (int)(rad*RAD2DEGREE);
                    sta->steer_angle = turn_degree;
                }
                else if ((Left_wheel_speed - Right_wheel_speed)>10 &&Right_wheel_speed>=0)
                {
                    sta->action_sta = NTZX_MC_ACTION_TURN_RIGHT;
                    sta->speed_mm  = Left_wheel_speed;
                    float rad = ((float) (Left_wheel_speed - Right_wheel_speed))/CAR_LEN;
                    int  turn_degree = (int)(rad*RAD2DEGREE);
                    sta->steer_angle = turn_degree; 
                }
                else if ( Left_wheel_speed < 0 && Right_wheel_speed >0  )
                {
                    sta->action_sta = NTZX_MC_ACTION_TWIRL_RIGHT;
                    sta->speed_mm = Right_wheel_speed;
                }
                else if( Left_wheel_speed >0 && Right_wheel_speed <0)
                {
                    sta->action_sta = NTZX_MC_ACTION_TWIRL_LEFT;
                    sta->speed_mm = Left_wheel_speed;
                }
                //留着准备给其他状态

                // //4为电压，5为小车的状态
                sta->vol = (row[i + 16] * 256 + row[i + 15]);
                sta->chassis_sta= row[i + 20] * 256 + row[i + 19];

                return NTZX_MC_SUCCESS;
            }
        }
    }

    // 未找到报文开发
    return NTZX_MC_RECV_INFO_ERR;
}