/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    sapient_service.cpp
 * @brief   SAPIENT 服务层实现
 *****************************************************************************/
#include "sapient_service.h"
#include "../sapientpb/sapient_init.h"
#include "../sapientpb/sapient_tcp.h"
#include "../protocol/protocol_object.h"
#include "../../common/zlog/skyfend_log.h"
#include <string.h>

#define LOG_TAG "sapient_service"

class CSapientService::Impl {
public:
    bool m_initialized;
    
    Impl() : m_initialized(false) {}
    
    ~Impl() {
        if (m_initialized) {
            sapient_cleanup();
        }
    }
};

CSapientService& CSapientService::GetInstance()
{
    static CSapientService instance;
    return instance;
}

int CSapientService::Init()
{
    if (m_impl && m_impl->m_initialized) {
        radar_log_warn("SAPIENT service already initialized");
        return 0;
    }
    
    if (!m_impl) {
        m_impl = std::make_unique<Impl>();
    }
    
    int ret = sapient_init();
    if (ret == 0) {
        m_impl->m_initialized = true;
        radar_log_info("SAPIENT service initialized successfully");
    } else if (ret == SAPIENT_ERR_NOT_CONFIGURED) {
        radar_log_info("SAPIENT not configured, service disabled");
    } else {
        radar_log_error("SAPIENT service initialization failed: %d", ret);
    }
    
    return ret;
}

void CSapientService::Cleanup()
{
    if (m_impl && m_impl->m_initialized) {
        sapient_cleanup();
        m_impl->m_initialized = false;
        radar_log_info("SAPIENT service cleaned up");
    }
}

int CSapientService::SendDetectionReport(const struct protocol_object_item_detected* target __attribute__((unused)))
{
    if (!m_impl || !m_impl->m_initialized) {
        return -1;
    }
    
    sapient_tcp_client_t* client = get_sapient_client();
    if (!client) {
        radar_log_error("SAPIENT client not available");
        return -1;
    }
    
    // 此函数已废弃，不再支持发送基于 protocol_object_item_detected 的检测报告
    // 请使用 CSapientAdapter::OnRadarTrack() 发送基于 RadarTrackItem 的检测报告
    radar_log_error("SendDetectionReport() is deprecated. Use CSapientAdapter::OnRadarTrack() instead.");
    return -1;
}

int CSapientService::SendStatusReport()
{
    if (!m_impl || !m_impl->m_initialized) {
        return -1;
    }
    
    sapient_tcp_client_t* client = get_sapient_client();
    if (!client) {
        radar_log_error("SAPIENT client not available");
        return -1;
    }
    
    int ret = sapient_tcp_client_send_status_report(client);
    if (ret != 0) {
        radar_log_error("Failed to send status report: %d", ret);
    }
    
    return ret;
}

int CSapientService::SendAlertReport(const char* description, int type, int status)
{
    if (!m_impl || !m_impl->m_initialized) {
        return -1;
    }
    
    sapient_tcp_client_t* client = get_sapient_client();
    if (!client) {
        radar_log_error("SAPIENT client not available");
        return -1;
    }
    
    int ret = sapient_tcp_client_send_alert_report(client, description, type, status);
    if (ret != 0) {
        radar_log_error("Failed to send alert report: %d", ret);
    }
    
    return ret;
}

bool CSapientService::IsConnected() const
{
    if (!m_impl || !m_impl->m_initialized) {
        return false;
    }
    
    sapient_tcp_client_t* client = get_sapient_client();
    return client != nullptr;
}

