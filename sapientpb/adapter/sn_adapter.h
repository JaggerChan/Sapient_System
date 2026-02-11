/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    sn_adapter.h
 * @brief   适配序列号读取接口
 *****************************************************************************/
#ifndef __SN_ADAPTER_H__
#define __SN_ADAPTER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SN_MAX_SIZE (25)

/**
 * @brief 读取设备序列号
 * @param sn 输出缓冲区
 * @param size 缓冲区大小
 * @return >0: 序列号长度, <0: 错误
 */
int read_sn(char *sn, int size);

#ifdef __cplusplus
}
#endif

#endif /* __SN_ADAPTER_H__ */

