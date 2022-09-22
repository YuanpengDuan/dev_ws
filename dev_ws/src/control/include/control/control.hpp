#pragma once 
#pragma pack(push) //保存对齐状态
#pragma pack(1)    //按1字节对齐

#define   CAR_LEN       (620)    //车子的宽度，单位mm
#define   RAD2DEGREE    (57.3)   //弧度转换成角度的系数
#define   ROWLENGTH     (64)	 //定义接收原始数据长度

typedef struct control_message
{
    int v_c; //前进速度 mm/s
    int w_c; //转向速度 rad/s

    //右轮速度 传入前进速度，转向速度，车的长度
    int v_r (int v_c, int w_c, int car_len){
    double rspeed;
    rspeed = v_c + w_c * car_len  / 2;
    return rspeed;
    }

    //左轮速度 传入前进速度，转向速度，车的长度
    int v_l (int v_c, int w_c, int car_len){
    double lspeed;
    lspeed = v_c - w_c * car_len / 2;
    return lspeed;
    }
}ctrl_m;

//上位机往机器人发送查询数据命令
//Cmd:0000 (查询数据 0x0000)
//查询产品硬件数据,机器人将返回包括产品类型、编码器线、电机最大转数、轮子 X轴距、轮子 Y 轴距、减速比、轮子直径等数据
//命令示例:ED DE 0C 00 00 00 01 00 D8 01 0D 0A
typedef struct Query_data_computer_to_robot
{
    int Head_first = 0xED; //协议头第一位
    int Head_second = 0xDE; //协议头第二位
    int Len = 0x0C; //协议长度
    int Type = 0x00; //产品类型
    int Cmd_low = 0x00; //命令低位
    int Cmd_high = 0x00; //命令高位
    int Num = 0x01; //数据个数
    int Data; //实际数据
    int Check_first = 0xD8; //校验码第一位
    int Check_second = 0x01; //校验码第二位
    int End_first = 0x0D; //结尾第一位
    int End_second = 0x0A;//结尾第二位
}Query_data_ctor;

//上位机往机器人发送开启上传命令
//Cmd:8007 (开启上传 0x8007)
//开启机器人 20ms 上传数据
//命令示例:ED DE 0B 04 07 80 00 61 02 0D 0A
typedef struct Enable_upload_computer_to_robot
{
    int Head_first = 0xED; //协议头第一位
    int Head_second = 0xDE; //协议头第二位
    int Len = 0x0B; //协议长度
    int Type = 0x04; //产品类型
    int Cmd_low = 0x07; //命令低位
    int Cmd_high = 0x80; //命令高位
    int Num = 0x00; //数据个数
    int Check_first = 0x61; //校验码第一位
    int Check_second = 0x02; //校验码第二位
    int End_first = 0x0D; //结尾第一位
    int End_second = 0x0A;//结尾第二位
}Enable_upload_ctor;

//上位机往机器人发送设置速度命令
//Cmd:8002 (设置速度 0x8002)
//设置机器人四个轮子速度
//命令示例:ED DE 17 04 02 80 06 50 01 50 01 50 01 50 01 00 00 00 00 B2 03 0D 0A
typedef struct Set_speed_computer_to_robot
{
    int Head_first = 0xED; //协议头第一位
    int Head_second = 0xDE; //协议头第二位
    int Len = 0x17; //协议长度
    int Type = 0x04; //产品类型
    int Cmd_low = 0x02; //命令低位
    int Cmd_high = 0x80; //命令高位
    int Num = 0x06; //数据个数
    int Front_right_speed_low; //前右轮速度低字节
    int Front_right_speed_higt; //前右轮速度高字节
    int Front_left_speed_low; //前左轮速度低字节
    int Front_left_speed_higt; //前左轮速度高字节
    int Rear_right_speed_low; //后右轮速度低字节
    int Rear_right_speed_higt; //后右轮速度高字节
    int Rear_left_speed_low; //后左轮速度低字节
    int Rear_left_speed_higt; //后左轮速度高字节
    int Placeholder1 = 0x00; //占位符1
    int Placeholder2 = 0x00; //占位符2
    int Placeholder3 = 0x00; //占位符3
    int Placeholder4 = 0x00; //占位符4
    int Check_first = 0xB2; //校验码第一位
    int Check_second = 0x03; //校验码第二位
    int End_first = 0x0D; //结尾第一位
    int End_second = 0x0A;//结尾第二位
}Set_speed_ctor;

//机器人往上位机发送返回数据命令
//Cmd:0000 (返回数据 0x0000)
//查询命令 Cmd :0000 的返回
//命令示例:ED DE 19 04 00 00 07 04 00 E8 03 B8 0B 00 00 00 00 34 00 40 01 16 04 0D 0A
typedef struct Return_data_robot_to_computer
{
    int Head_first = 0xED; //协议头第一位
    int Head_second = 0xDE; //协议头第二位
    int Len = 0x19; //协议长度
    int Type = 0x04; //产品类型
    int Cmd_low = 0x00; //命令低位
    int Cmd_high = 0x00; //命令高位
    int Num = 0x07; //数据个数
    int Type_low; //产品类型低字节
    int Type_high; //产品类型高字节
    int Encoder_lines_low; //编码器线数低字节
    int Encoder_lines_high; //编码器线数高字节
    int Revolutions_maximum_low; //电机最大转数低字节
    int Revolutions_maximum_higt; //电机最大转数高字节
    int X_wheelbase_low; //轮子 X 轴轴距低字节
    int X_wheelbase_higt; //轮子 X 轴轴距高字节
    int Y_wheelbase_low; //轮子 Y 轴轴距低字节
    int Y_wheelbase_higt; //轮子 Y 轴轴距高字节
    int Reduction_ratio_low; //电机减速比低字节
    int Reduction_ratio_higt; //电机减速比高字节
    int Diameter_low; //轮子直径(mm)低字节
    int Diameter_higt; //轮子直径(mm)高字节
    int Check_first = 0x16; //校验码第一位
    int Check_second = 0x04; //校验码第二位
    int End_first = 0x0D; //结尾第一位
    int End_second = 0x0A;//结尾第二位
}Return_data_ctor;

//机器人往上位机发送上传命令
//Cmd:8007 (20ms 上传 0x8007)
//20ms 上传的机器人信息
//命令示例: ED DE 19 04 07 80 07 00 00 00 00 00 00 00 00 00 FF 00 00 00 00 75 03 0A 0D
typedef struct Upload_computer_to_robot
{
    int Head_first = 0xED; //协议头第一位
    int Head_second = 0xDE; //协议头第二位
    int Len = 0x19; //协议长度
    int Type = 0x04; //产品类型
    int Cmd_low = 0x07; //命令低位
    int Cmd_high = 0x80; //命令高位
    int Num = 0x07; //数据个数
    int Front_right_speed_low; //前右轮速度低字节
    int Front_right_speed_higt; //前右轮速度高字节
    int Front_left_speed_low; //前左轮速度低字节
    int Front_left_speed_higt; //前左轮速度高字节
    int Rear_right_speed_low; //后右轮速度低字节
    int Rear_right_speed_higt; //后右轮速度高字节
    int Rear_left_speed_low; //后左轮速度低字节
    int Rear_left_speed_higt; //后左轮速度高字节
    int Voltage_low; //电压数据低字节
    int Voltage_high; //电压数据高字节
    int Placeholder1 = 0x00; //占位符1
    int Placeholder2 = 0x00; //占位符2
    int MC_low; //底盘状态低字节
    int MC_higt; //底盘状态高字节
    int Check_first = 0x75; //校验码第一位
    int Check_second = 0x03; //校验码第二位
    int End_first = 0x0D; //结尾第一位
    int End_second = 0x0A;//结尾第二位
}Upload_ctor;

#pragma pack(pop)//恢复对齐状态