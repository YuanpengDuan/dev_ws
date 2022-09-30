#ifndef _MC_STA_H_
#define _MC_STA_H_

/* 关于参数action 各项值的定义,unsigned char action 的高3位没有用到，暂时保留 */
#define NTZX_MC_ACTION_NULL			     	      0x00	//0x00：表示无操作
#define	NTZX_MC_ACTION_STOP  			          0x01	//0x01：表示汽车停止，action的最低位
#define	NTZX_MC_ACTION_GO_FORWARD 	 0x02	//0x02：表示汽车前行，action的次低位
#define	NTZX_MC_ACTION_GO_BACKED	        0x04	//0x04： 表示汽车倒车
#define	NTZX_MC_ACTION_TURN_LEFT		     0x08	//0x08：  表示汽车左拐
#define NTZX_MC_ACTION_TURN_RIGHT	   0x10	//0x10：表示汽车右拐
#define	NTZX_MC_ACTION_TWIRL_LEFT		 0x20	//0x08：表示汽车原地左转
#define NTZX_MC_ACTION_TWIRL_RIGHT      0x40	//0x10：表示汽车原地右转

#define   CAR_LEN       (620)    //车子的宽度，单位mm
#define   RAD2DEGREE    (57.3)   //弧度转换成角度的系数
#define   ROWLENGTH     (64)	 //定义接收原始数据长度
#define   SPEEDMAX       1000   // 定义一个最大值
#define   MC_RECV_LEN   64
#define   MC_RECV_STA_PERIOD    50000

/*车辆反馈结构体*/
typedef struct mc_state_info{
    unsigned char action_sta;   // 运动状态
    unsigned int io_sta;        // io状态
    short speed_mm;             // 速度
    short steer_angle;          // 转向角
    unsigned short vol;         // 电压
    unsigned short chassis_sta; // 底盘状态
}struct_mc_sta;

#endif