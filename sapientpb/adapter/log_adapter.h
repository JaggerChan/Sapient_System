/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    log_adapter.h
 * @brief   适配 log.h 接口（使用 acur101 的 zlog）
 *****************************************************************************/
#ifndef __LOG_ADAPTER_H__
#define __LOG_ADAPTER_H__

// 将 sfl100_ps 的 log.h 接口映射到 acur101 的 skyfend_log.h
#include "../../common/zlog/skyfend_log.h"

// 定义兼容的宏
#define LOG_TAG "sapient"

// 如果代码中使用了 LOGE, LOGI, LOGW, LOGD，这些已经在 skyfend_log.h 中定义了
// 如果没有，可以在这里定义兼容宏

#endif /* __LOG_ADAPTER_H__ */


