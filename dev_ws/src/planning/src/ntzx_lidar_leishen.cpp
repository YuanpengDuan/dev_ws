/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_lidar_leishen.c
 * 作者：李进
 * 日期：2021年2月26日
 * 描述：3D激光雷达数据的处理和接收
 * ******************************************************************/

#include "planning/ntzx_lidar_leishen.hpp"
#include "planning/ntzx_conf_app.hpp"
#include "planning/ntzx_log_app.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>


#define LEISHEN_LIDAR_TITLE     "LIDAR"
#define LEISHEN_LIDAR_PROT_KEY  "PORT"
#define LIDAR_16LINE    16
#define LIDAR_32LINE    32

static int g_leishen_udp_port = -1;
static int g_lidar_leishen_fd = -1;

static unsigned char g_leishen_udp_packet[LIDAR_LEISHEN_UDP_PACKET_SIZE] = {0};
static lenshen_3D16_Point m_DecodePoint[LIDAR_MAX_ONE_FRAME_POINT_NUM];//存储未完成的一帧图像中所有的点
static lenshen_3D16_Point m_OneFramePoint[LIDAR_MAX_ONE_FRAME_POINT_NUM];//存储未完成的一帧图像中所有的点
static int m_IsIgnore[LIDAR_MAX_ONE_FRAME_POINT_NUM];//是否判断为悬浮障碍物而需要被忽视的点
static int m_UsedAngle[LIDAR_ONE_ROUND_ANGLE_NUM];
static pthread_mutex_t g_L0_Mutex_NewPacket = PTHREAD_MUTEX_INITIALIZER;//用于控制收发UDP包的线程
static int g_L0_ReceivedPacketNum = 0; //总共接收到的UDP包数 用于各线程间判断新包到达 初始为0
static int g_L0_OneFramePacketSize[LIADR_MAX_ONE_FRAME_PACKET];//记录一帧数据中，每个数据包的大小
static unsigned char g_L0_OneFrameBuf[LIADR_MAX_ONE_FRAME_PACKET][LIDAR_MAX_UDP_PACKET_SIZE];//存放一帧数据，最多84包,每包大小为(1248+100)
static int m_Group = 0;   //目前的数据中垂直线个数，垂直线最大值2000，初始值为0。
static int m_Angle[LIDAR_MAX_ONE_FRAME_GROUP_NUM];//存储一条垂直线对应的角度
static int m_preAngle; //当前角度的上一个角度值，用于判断是否满足一帧（是否成环）
static int m_IsFirstAngel;//一个角度是否是一帧中的第一个角度
static int m_IsNewFrame;//判断是否产生了新的一帧数据
static int g_L0_RecivedFrameNum = 0; //总共提取出的帧数
static int m_PointNumInFrame;//一个完整的帧中点的数量

static double m_calib_VertC[LIDAR_16LINE] = {-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15};  //垂直角度参数修正
static double m_cosVetCorrection[LIDAR_16LINE]; //极坐标系转化成笛卡尔坐标系时使用
static double m_sinVetCorrection[LIDAR_16LINE]; //。。。
//与栅格相关
static int OBS_PointNum_GridMsk[LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM];//记录栅格的点的数量
static int OBS_MaxHeight_GridMsk[LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM];//记录栅格中最大高度
static int OBS_MinHeight_GridMsk[LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM];//记录栅格中最小高度
static unsigned char Is_OBS_GridMsk[LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM];//障碍物栅格
static pthread_mutex_t g_mutex_lidar_leishen_info = PTHREAD_MUTEX_INITIALIZER;
static str_lidar_info_to_fuse g_lidar_to_fuse = {0};
static int lidar_frame_to_fuse = 0;

static int ntzx_lidar_leishen_init(void);
static void *ntzx_lidar_leishen_recv(void *recvbuf);
static void *ntzx_lidar_leishen_proc(void *recvbuf);
static void ntzx_lidar_leishen_depack(unsigned char*nPacket);
static void ntzx_lidar_IsNewFrame(int cur_angle);
static void ntzx_lidar_SaveOneFrame(void);
static void ntzx_lidar_obs_init(void);
static void Convert_Car_XYZ(int *x, int *y, int*z);
static void Convert_Grid_XYZ(int *x, int *y, int*z);
static void ntzx_lidar_obs_create_grid(lenshen_3D16_Point*Frame, int PointNum);

static int ntzx_lidar_leishen_init(void)
{
    // 创建UDP通信套接字
    int rt = -1;
    char buf_temp[10] = {0};
    int nRecvBufLen = 0;
    int opt = 1;
    int iRecvTimeOut = 80;
    struct sockaddr_in my_addr;                 // my address information
    /* 初始化值 */
    for (int i = 0; i < LIDAR_16LINE; i++) {
        m_cosVetCorrection[i] = cos(m_calib_VertC[i]  * PIDIVIDED) ;//1.8为雷达的角度校正,
        m_sinVetCorrection[i]  = sin(m_calib_VertC[i] * PIDIVIDED)   ;
    }
    ntzx_get_conf_key_string(LEISHEN_LIDAR_TITLE, LEISHEN_LIDAR_PROT_KEY, buf_temp, sizeof(buf_temp) - 1);
    // g_leishen_udp_port = atoi(buf_temp);
    g_leishen_udp_port = 2368;
    printf("g_leishen_udp_port=%d",g_leishen_udp_port);
    if (g_leishen_udp_port <= 0) {
        return NTZX_LIDAR_LEISHEN_INIT_ERR;
    }
    g_lidar_leishen_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_lidar_leishen_fd < 0) {
        return NTZX_LIDAR_LEISHEN_INIT_ERR;
    }
    // 设置允许重用本地地址和端口
    setsockopt(g_lidar_leishen_fd, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(int)); //sizeof(opt)
    // 设置换冲过去大小为0
    setsockopt(g_lidar_leishen_fd, SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBufLen, sizeof(int));
    // 设置接收超时
    setsockopt(g_lidar_leishen_fd, SOL_SOCKET, SO_RCVTIMEO, &iRecvTimeOut, sizeof(int));
    
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(g_leishen_udp_port);    // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP、
    
    // 绑定IP和端口号
    rt = bind(g_lidar_leishen_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr));  // 将sockaddr_in转化成socketaddr类型
    if (rt < 0) {
        close(g_lidar_leishen_fd);
        return NTZX_LIDAR_LEISHEN_BIND_ERR;
    }
    return NTZX_LIDAR_LEISHEN_SUCCESS;
}

void *ntzx_lidar_leishen_main(void *arg)
{
    
    int rt;
    pthread_t lidar_leishen_recv;
    pthread_t lidar_leishen_handle;
    rt = ntzx_lidar_leishen_init();
    printf("rt = %d",rt);
    if (rt < 0) {
        ntzx_stereotypes_log_write_err("lidar init fail:", rt);
        return NULL;
    }
    /* 创建两个线程，接收线程与处理线程 */
    rt = pthread_create(&lidar_leishen_recv, NULL, ntzx_lidar_leishen_recv, g_leishen_udp_packet);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("lidar creat recv thread err!!!");
        return NULL;
    }
    rt = pthread_create(&lidar_leishen_handle, NULL, ntzx_lidar_leishen_proc, g_leishen_udp_packet);
    if (rt < 0) {
        ntzx_stereotypes_log_write_buf("lidar creat proc thread err!!!");
        return NULL;
    }
    pthread_join(lidar_leishen_recv, NULL);
    pthread_join(lidar_leishen_handle, NULL);
    return NULL;
}

void *ntzx_lidar_leishen_recv(void *recvbuf)
{
    printf("ntzx_lidar_leishen_recv");
    int rt = -1;
    struct sockaddr_in server_addr;
    socklen_t server_len;
    int PacketLen;//正确接收的数据包中的字节数
    int idx;//对接收的数据包编号
    int SleepTime;//连续睡眠时间计数
    SleepTime = 0;

    while (1) {
        //step.1---------------接收一个网络包-------------------------//
        server_len = sizeof(server_addr);
        PacketLen = recvfrom(g_lidar_leishen_fd, (unsigned char *)recvbuf, LIDAR_LEISHEN_UDP_PACKET_SIZE, 0,(struct sockaddr*) &server_addr, &server_len);//接收数据和发送方的地址
        //step.2---------------存放本包数据--------------------------- //
        if (PacketLen == LIDAR_LEISHEN_UDP_PACKET_SIZE) //只接受UDP报文，雷神UDP报文设置大小为1206
        {
            SleepTime = 0;//睡眠计数清零
            pthread_mutex_lock(&g_L0_Mutex_NewPacket);
            idx = g_L0_ReceivedPacketNum % LIADR_MAX_ONE_FRAME_PACKET;
            memcpy(&(g_L0_OneFrameBuf[idx][0]), recvbuf, PacketLen);
            g_L0_OneFramePacketSize[idx] = PacketLen;
            g_L0_ReceivedPacketNum++;
            pthread_mutex_unlock(&g_L0_Mutex_NewPacket);
        }
        //step.3---------------稍等,取下一包--------------------------//
        else {
            usleep(LIDAR_PACKET_INTERVAL_TIME);
            SleepTime++;
            //长时间收不到包就重新启动激光雷达
            if (SleepTime > 100) {
                rt = ntzx_lidar_leishen_init();
                if (rt == NTZX_LIDAR_LEISHEN_SUCCESS) {
                    SleepTime = 0;
                }
            }
        }
    }
}

void *ntzx_lidar_leishen_proc(void *recvbuf)
{
    printf("ntzx_lidar_leishen_proc");
    int rt;
    int nTransformed,nReceived; //nTransformed 可能会溢出
    int nBytes;
    int idx;
    unsigned char nPacket[LIDAR_MAX_UDP_PACKET_SIZE]; //数据包暂存
    nTransformed = 0;
    nBytes = 0;
    while (1) 
    {
        usleep(50);
        pthread_mutex_lock(&g_L0_Mutex_NewPacket);
        nReceived = g_L0_ReceivedPacketNum;
        //判断是否有新包,无新包时,等待
        if (nTransformed == nReceived) {
            usleep(LIDAR_PACKET_INTERVAL_TIME);
            pthread_mutex_unlock(&g_L0_Mutex_NewPacket);
            continue;
        }
        idx = nTransformed % LIADR_MAX_ONE_FRAME_PACKET;
        nBytes = g_L0_OneFramePacketSize[idx];
        memcpy(nPacket, &(g_L0_OneFrameBuf[idx][0]), nBytes);
        pthread_mutex_unlock(&g_L0_Mutex_NewPacket);
        nTransformed++;
        //对一包数据进行解码,并判断是否形成了新一帧数据
        ntzx_lidar_leishen_depack(nPacket);
        //判断是否满足一帧，是则存储，否则继续接收数据
        if (m_IsNewFrame) 
        {
            g_L0_RecivedFrameNum++;
            ntzx_lidar_SaveOneFrame();
            //显示障碍点
            ntzx_lidar_obs_init();
            ntzx_lidar_obs_create_grid(m_OneFramePoint, m_PointNumInFrame);
            m_Group = 0;
            m_IsNewFrame = 0;
            memset(m_UsedAngle, 0, sizeof(int) * LIDAR_ONE_ROUND_ANGLE_NUM);
            memset(m_Angle, 0, sizeof(int) * LIDAR_MAX_ONE_FRAME_GROUP_NUM);
            memset(m_IsIgnore, 0, sizeof(int) * LIDAR_MAX_ONE_FRAME_POINT_NUM);
            m_IsFirstAngel = 1;//一个角度是否是一帧中的第一个角度
            //指向最新一包数据
            pthread_mutex_lock(&g_L0_Mutex_NewPacket);
            nTransformed = g_L0_ReceivedPacketNum;
            pthread_mutex_unlock(&g_L0_Mutex_NewPacket);
        }
    }
}

int ntzx_lidar_leishen_get_data(str_lidar_info_to_fuse *lidar_data)
{
    if (lidar_frame_to_fuse != g_lidar_to_fuse.frame_id) {
        pthread_mutex_lock(&g_mutex_lidar_leishen_info);
        memcpy(lidar_data, &g_lidar_to_fuse, sizeof(str_lidar_info_to_fuse));
        lidar_frame_to_fuse = g_lidar_to_fuse.frame_id;
        pthread_mutex_unlock(&g_mutex_lidar_leishen_info);
    } else {
        return NTZX_LIDAR_LEISHEN_DATA_ERR;
    }
    return NTZX_LIDAR_LEISHEN_SUCCESS;
}

static void ntzx_lidar_leishen_depack(unsigned char*nPacket)
{
    GROUP cur;
    lenshen_3D16_Point nPoint3D;//存储一个点的信息
    int IsOdd;//在一个扇区中是否位于第一包，是不是每一块数据的第一组
    //初始化变量
    IsOdd = 1;//位置初始为扇区中的第一部分
    memset(&nPoint3D, 0, sizeof(lenshen_3D16_Point));
    //开始对一包数据进行解析
    int temp_i = LIDAR_LEISHEN_UDP_PACKET_SIZE - LIDAR_PACKET_TAIL_LEN - LIDAR_LENGTH_OF_SECOND_PART_OF_ONE_BLOCK;//1190，其中48 = 16 *3 
    for (int i = 0;i <= temp_i;) 
    {  
        //求解水平角度
        if (IsOdd) {
            cur.angle = nPacket[i + 3] *256 + nPacket[i + 2];//每块第一组角度，应该是小端模式
        }
        else {
            cur.angle += LIDAR_HANGLE_RESOLUTION * 100;//雷达扫描相邻的两条垂直线差0.18/0.36度，因为用的雷达频率是10/20HZ
        }
        //过滤车后方无用的点
        if (0) {
            if (IsOdd) {
                i += LIDAR_LENGTH_OF_FIRST_PART_OF_ONE_BLOCK;
                IsOdd = 0;
            } else {
                i += LIDAR_LENGTH_OF_SECOND_PART_OF_ONE_BLOCK;
                IsOdd = 1;
            }
            continue;
        }
        nPoint3D.angle = cur.angle;
        for (int j = 0; j < NUM_OF_LASERS; j++) 
        {   //j代表垂直方向上的激光发射器编号
            //距离，看一下雷达的说明文档中UDP包的结构就明白3,4,5是什么意思了
            if (IsOdd) {
                cur.distance[j] = (nPacket[j * 3 + i + 5] * 256 + nPacket[j * 3 + i + 4]) / 4;  //因为单位是0.25cm，所以最后除以4
            } else {
                cur.distance[j] = (nPacket[j * 3 + i + 1] * 256 + nPacket[j * 3 + i + 0]) / 4;  
            }
            //过滤，25cm近和150m远去掉
            if ((cur.distance[j] < LIDAR_MIN_DETECT_DISTANCE) || (cur.distance[j] > LIDAR_MAX_DETECT_DISTANCE)) {
                continue;
            }
            //回波强度
            float intense;
            if (IsOdd) {
                intense = (float)nPacket[j * 3 + i + 6];//提取反射率信息
            } else {
                intense = (float)nPacket[j * 3 + i + 2];//提取反射率信息
            }
            //坐标线转换
            double rotAngle = (double)(cur.angle * 0.01);
            double cosRotAngle = (double)cos(rotAngle * PIDIVIDED);
            double sinRotAngle = (double)sin(rotAngle * PIDIVIDED);
            double cosVertAngle = m_cosVetCorrection[j];
            double sinVertAngle = m_sinVetCorrection[j];
            //XYZ坐标
            double x = cur.distance[j] * cosVertAngle * sinRotAngle;
            double y = cur.distance[j] * cosVertAngle * cosRotAngle;
            double z = cur.distance[j] * sinVertAngle;

            //存储
            nPoint3D.scanID = j;
            nPoint3D.x = x;//以cm为单位
            nPoint3D.y = y;//以cm为单位
            nPoint3D.z = z;//以cm为单位
            nPoint3D.d = cur.distance[j];//以cm为单位
            nPoint3D.intense = intense;
            memcpy(&m_DecodePoint[j * LIDAR_MAX_ONE_FRAME_GROUP_NUM + m_Group], &nPoint3D, sizeof(lenshen_3D16_Point));
        }
        if (IsOdd) {
            i += LIDAR_LENGTH_OF_FIRST_PART_OF_ONE_BLOCK;//跳到到一组数据的第二个角度，48+52=100，就是UDP包中一组数据的长度，一组数据包含了两个角度的点，可以参见雷达的说明文档
            IsOdd = 0;
        } else {
            i += LIDAR_LENGTH_OF_SECOND_PART_OF_ONE_BLOCK;
            IsOdd = 1;
        }
        m_Angle[m_Group] = cur.angle;
        m_Group++;
        ntzx_lidar_IsNewFrame(cur.angle);
        if (m_IsNewFrame) {
            return;
        }
    }
    return;
}

static void ntzx_lidar_IsNewFrame(int cur_angle)
{
    if (m_IsFirstAngel) {
        m_UsedAngle[cur_angle] = 1;
        m_preAngle = cur_angle;
        m_IsFirstAngel = 0;
    } else {
        int i = (cur_angle + LIDAR_ONE_ROUND_ANGLE_NUM - m_preAngle) % LIDAR_ONE_ROUND_ANGLE_NUM;
        for (int j = 1; j <= i; j++) {
            int pos = (m_preAngle + j) % LIDAR_ONE_ROUND_ANGLE_NUM;
            if (m_UsedAngle[pos]) {
                m_IsNewFrame = 1;
                return;
            }
            m_UsedAngle[pos] = 1;
        }
        //上一次
        m_preAngle = cur_angle;
    }
    m_IsNewFrame = 0;
    return;
}

static void ntzx_lidar_SaveOneFrame(void)
{
    int min_dif, dif;
    int StartGroup;
    int pos;
    int pre, cur;//悬浮障碍判断时指示当前雷达扫描线和上次记录的有效雷达扫描线
    //初始化
    m_PointNumInFrame = 0;
    pos = 0;

    //寻找距离18000最小的角度作为起始点
    dif = abs(m_Angle[0] - LIDAR_TRANSFORM_START_ANGLE);
    min_dif = dif;
    for (int i = 1; i < m_Group; i++)
    {
        dif = abs(m_Angle[i] - LIDAR_TRANSFORM_START_ANGLE);
        if (dif < min_dif)
        {
            StartGroup = i;
            min_dif = dif;
        }
    }
    
    //过滤悬浮障碍
    for (int i = 0; i < m_Group; i++) {
        pre = m_DecodePoint[i].x * m_DecodePoint[i].x + m_DecodePoint[i].y * m_DecodePoint[i].y;
        //雷达线的顺序 从下到上为1,3,5,7,9,11,13,15,2,4,6,8,10,12,14,16，具体可参见雷达手册
        for (int j = 2; j < 16; j += 2) {
            pos = j * LIDAR_MAX_ONE_FRAME_GROUP_NUM + i;
            cur = m_DecodePoint[pos].x*m_DecodePoint[pos].x + m_DecodePoint[pos].y*m_DecodePoint[pos].y;
            if (cur < pre) {
                m_IsIgnore[pos] = 1;
            }
            else {
                pre = cur;
            }
        }
        for (int j = 1; j < 16; j += 2) {
            pos = j * LIDAR_MAX_ONE_FRAME_GROUP_NUM + i;
            cur = m_DecodePoint[pos].x*m_DecodePoint[pos].x + m_DecodePoint[pos].y*m_DecodePoint[pos].y;
            if (cur < pre) {
                m_IsIgnore[pos] = 1;
            }
            else {
                pre = cur;
            }
        }
    }
    //从起始点重新组织数据
    //exchange（i,j）将会导致错误->floating num exception
    for (int i = 0; i < NUM_OF_LASERS; i++) {
        for (int j = 0; j < m_Group; j++)
    	{
			pos = (StartGroup + j) % m_Group;//水平方向偏移
			pos += i * LIDAR_MAX_ONE_FRAME_GROUP_NUM;//垂直方向偏移
			//如果改点是需要被忽略的点（悬浮障碍）||XYZ坐标均为0
			if (m_IsIgnore[pos] || (m_DecodePoint[pos].x == 0 && m_DecodePoint[pos].y == 0 && m_DecodePoint[pos].z == 0))
				continue;
			else{
				memcpy(&m_OneFramePoint[m_PointNumInFrame], &m_DecodePoint[pos], sizeof(lenshen_3D16_Point));
				m_PointNumInFrame++;
			}
		}
    }
    return;
}

static void ntzx_lidar_obs_init(void)
{
    memset(OBS_PointNum_GridMsk, 0, sizeof(int) * LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM);
    memset(Is_OBS_GridMsk, 0, sizeof(unsigned char)* LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM);
    for (int i = 0; i < LIDAR_VERTICAL_GRID_NUM * LIDAR_HORIZONTAL_GRID_NUM; i++) {
        OBS_MaxHeight_GridMsk[i] = -2147483648;
        OBS_MinHeight_GridMsk[i] = 2147483647;
    }
}

static void ntzx_lidar_obs_create_grid(lenshen_3D16_Point*Frame, int PointNum)
{
    int point_x, point_y, point_z;
    //update grid
    for (int i = 0; i < PointNum; i++)
    {
        point_x = (int)Frame[i].x;
        point_y = (int)Frame[i].y;
        point_z = (int)Frame[i].z;
        //雷达坐标系转车体坐标系
        Convert_Car_XYZ(&point_x, &point_y, &point_z);
        //剔除不在栅格范围内的点
        if (point_x >= LIDAR16_RIGHT_CAR_DISTANCE_CM ||        
            point_x <= -LIDAR16_LEFT_CAR_DISTANCE_CM ||		   
            point_y >= LIDAR16_BREFORE_CAR_DISTANCE_CM ||	   
            point_y <= -LIDAR16_AFTER_CAR_DISTANCE_CM||		   
            point_z >= HEIGHT_THRESHOLD_MAX)		               //2400/过滤高障碍物(车上方树木）
        {
            continue;
        }
        Convert_Grid_XYZ(&point_x, &point_y, &point_z);
        //判断在栅格中的位置
        int p = point_x / LIDAR_VERTICAL_DISTANCE_RESOLUTION_CM; 
        int q = point_y / LIDAR_HORIZONTAL_DISTANCE_RESOLUTION_CM; 
        int position = p + q * LIDAR_HORIZONTAL_GRID_NUM; 
        //对栅格中点的数量和最大高度进行更新
        OBS_PointNum_GridMsk[position]++;
        if (point_z > OBS_MaxHeight_GridMsk[position])
        {
            OBS_MaxHeight_GridMsk[position] = point_z;
        }
        if (point_z < OBS_MinHeight_GridMsk[position])
        {
            OBS_MinHeight_GridMsk[position] = point_z;
        }

    }
    //根据阈值填充障碍物栅格
    for (int i = 0; i < LIDAR_GRID_ALL_NUM; i++)
    {
        if(OBS_MaxHeight_GridMsk[i] > 20 ) {//低于20不作为障碍(经验值)
            Is_OBS_GridMsk[i] = WRC_3D16_OBS_SURE;
        } else {
            Is_OBS_GridMsk[i] = WRC_3D16_OBS_NONE;
        }
    }
    //填充结构体GridToFu
    pthread_mutex_lock(&g_mutex_lidar_leishen_info);
    memcpy(g_lidar_to_fuse.gridmask,Is_OBS_GridMsk,sizeof(Is_OBS_GridMsk));
    g_lidar_to_fuse.frame_id = g_L0_RecivedFrameNum;
    pthread_mutex_unlock(&g_mutex_lidar_leishen_info);
}

static void Convert_Car_XYZ(int *x, int *y, int*z)
{
    int temp_x, temp_y, temp_z;
    temp_x = -0.0449 * (*x) + 0.9965* (*y) + 0.0142 * (*z) - 2.7219;//标定获得（惯导投影到地面）
    temp_y = -0.9949 * (*x) - 0.0432* (*y) + 0.2102 * (*z) + 62.3548;
    temp_z = 0.0169 * (*x) - 0.0082* (*y) + 0.8892 * (*z) + 110.6139;
    *x = temp_x;
    *y = temp_y;
    *z = temp_z;
}

static void Convert_Grid_XYZ(int *x, int *y, int*z)
{
    *x = *x + LIDAR16_LEFT_CAR_DISTANCE_CM;
    *y = LIDAR16_BREFORE_CAR_DISTANCE_CM - *y;
}