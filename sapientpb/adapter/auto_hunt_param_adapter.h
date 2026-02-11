/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    auto_hunt_param_adapter.h
 * @brief   适配 auto_hunt_param 接口（acur101 可能没有此模块）
 *****************************************************************************/
#ifndef __AUTO_HUNT_PARAM_ADAPTER_H__
#define __AUTO_HUNT_PARAM_ADAPTER_H__

#include "../../inc/GNSS_coordinate.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 GNSS 坐标（适配接口）
 * @param coordinate 输出参数，GNSS 坐标
 * @return 0 成功，-1 失败
 */
int auto_hunt_param_get_GNSS(GNSS_coordinate_t *coordinate);

#ifdef __cplusplus
}
#endif

#endif /* __AUTO_HUNT_PARAM_ADAPTER_H__ */


