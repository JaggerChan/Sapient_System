/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    auto_hunt_param_adapter.c
 * @brief   auto_hunt_param 适配实现
 *****************************************************************************/
#include "auto_hunt_param_adapter.h"
#include "../../common/zlog/skyfend_log.h"
// 注意：暂时不包含 attitude_report.h，因为它包含 C++ 头文件
// TODO: 当需要实际获取 GNSS 数据时，需要将本文件改为 .cpp 或创建 C 接口封装

#define LOG_TAG "auto_hunt_adapter"

int auto_hunt_param_get_GNSS(GNSS_coordinate_t *coordinate)
{
    if (!coordinate) {
        return -1;
    }

    // TODO: 从 acur101 的姿态服务获取 GNSS 坐标
    // 这里先返回默认值，后续需要集成实际的 GNSS 数据源
    coordinate->type = GNSS_COORDINATE_MYSELF;
    coordinate->longitude = 0;
    coordinate->latitude = 0;
    coordinate->altitude = 0;

    radar_log_debug("auto_hunt_param_get_GNSS: returning default values");
    return 0;
}


