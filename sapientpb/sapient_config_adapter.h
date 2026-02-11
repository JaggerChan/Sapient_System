/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    sapient_config_adapter.h
 * @brief   SAPIENT 配置适配器 - 适配 acur101 的配置系统
 * @details 提供与 sfl100_ps 的 dev_config 兼容的接口
 *****************************************************************************/
#ifndef __SAPIENT_CONFIG_ADAPTER_H__
#define __SAPIENT_CONFIG_ADAPTER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 配置结构体（兼容 dev_config 接口） */
typedef struct {
    const char *ip;
    int port;
} sapient_config_t;

/**
 * @brief 获取 SAPIENT 配置
 * @return sapient_config_t* 配置指针，如果未配置则返回 NULL
 */
const sapient_config_t *sapient_config_get(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAPIENT_CONFIG_ADAPTER_H__ */

