/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    sn_adapter.c
 * @brief   序列号读取适配实现
 *****************************************************************************/
#include "sn_adapter.h"
#include "../../common/zlog/skyfend_log.h"
#include "../../common/radar_common.h"  /* 从文件获取设备 SN */
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "radar_state_adapter.h"   /* 从 RadarState 获取设备 SN */

#define LOG_TAG "sn_adapter"

// TODO: 实现实际的序列号读取逻辑
// 这里提供一个临时实现，需要根据实际硬件平台进行适配
int read_sn(char *sn, int size)
{
    if (!sn || size <= 0) {
        radar_log_error("read_sn: invalid parameters");
        return -1;
    }

    /* 1) 首选：从 RadarState (msgid=0x20) 中读取雷达 SN  */
    RadarState state;
    memset(&state, 0, sizeof(RadarState));

    if (get_radar_state(&state) == 0 && state.has_sn) {
        /* RadarState.sn 是 char[32]，不保证以 '\0' 结尾，这里安全截断复制
         * 同时遵守 sn_adapter.h 中定义的 SN_MAX_SIZE 上限
         */
        int max_copy = (size - 1) > SN_MAX_SIZE ? SN_MAX_SIZE : (size - 1);
        int src_len = 0;
        while (src_len < max_copy && state.sn[src_len] != '\0') {
            ++src_len;
        }

        if (src_len > 0) {
            memcpy(sn, state.sn, src_len);
            sn[src_len] = '\0';
            radar_log_debug("read_sn: using SN from RadarState (len=%d)", src_len);
            return src_len;
        }
    }

    /* 2) 回退：如果当前还没有有效的 RadarState 或者 RadarState 中没有 SN，
     *    直接从序列号文件读取（通过 GetDeviceSN()）。
     *    这确保即使在 RadarState 准备好之前也能读取到正确的序列号。
     */
    const char *device_sn = GetDeviceSN();
    if (!device_sn) {
        radar_log_error("read_sn: GetDeviceSN() returned NULL");
        return -1;
    }
    
    int len = strlen(device_sn);
    if (len == 0) {
        radar_log_warn("read_sn: GetDeviceSN() returned empty string");
        return -1;
    }
    
    if (len >= size) {
        len = size - 1;
    }

    memcpy(sn, device_sn, len);
    sn[len] = '\0';

    radar_log_debug("read_sn: using SN from GetDeviceSN() '%s' (RadarState not ready yet)", device_sn);
    return len;
}

