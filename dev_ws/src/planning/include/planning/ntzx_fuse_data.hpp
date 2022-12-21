/********************************************************************
 * Copyright (C), 2021, 南通智行未来车联网创新中心有限公司
 * 文件名：ntzx_fuse_date.h
 * 作者：任家豪
 * 日期：2021年2月26日
 * 描述：数据融合的头文件
 * ******************************************************************/

#ifndef _NTZX_FUSE_DATE_
#define _NTZX_FUSE_DATE_

#include "ntzx_lidar_leishen.hpp"

#define NTZX_FUSE_SUCCESS   0
#define NTZX_FUSE_ERR   (-101)
#define NTZX_FUSE_INCOMPLETE (-102)
#define NTZX_FUSE_TRYLOCK_FAIL (-103)

#define NTZX_FUSE_GRID_EXIST    1
#define NTZX_FUSE_GRID_NOEXIST  0

#pragma pack(push, 1)
typedef struct grid_fuse_to_pl{
    int             frame_id;       // 雷达点云图(帧号)
    unsigned char   gridmask[LIDAR_VERTICAL_GRID_NUM*LIDAR_HORIZONTAL_GRID_NUM]; // 栅格点数据
}str_grid_fuse_to_pl;
#pragma pack(pop)

void *ntzx_fuse_data_main(void *arg);
int ntzx_get_grid_data(str_grid_fuse_to_pl *fuse_to_pl);
#endif