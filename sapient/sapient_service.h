/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    sapient_service.h
 * @brief   SAPIENT 服务层接口
 * @details 提供高级接口，封装底层 TCP 客户端
 *****************************************************************************/
#ifndef __SAPIENT_SERVICE_H__
#define __SAPIENT_SERVICE_H__

#include <string>
#include <memory>

#ifdef __cplusplus
extern "C" {
#endif

/* 前向声明 */
struct protocol_object_item_detected;

#ifdef __cplusplus
}
#endif

class CSapientService {
public:
    static CSapientService& GetInstance();
    
    // 初始化（从配置读取 IP/端口）
    int Init();
    
    // 清理资源
    void Cleanup();
    
    // 发送检测报告（使用 protocol_object_item_detected 结构）
    int SendDetectionReport(const struct protocol_object_item_detected* target);
    
    // 发送状态报告
    int SendStatusReport();
    
    // 发送告警报告
    int SendAlertReport(const char* description, int type, int status);
    
    // 检查连接状态
    bool IsConnected() const;
    
private:
    CSapientService() = default;
    ~CSapientService() = default;
    CSapientService(const CSapientService&) = delete;
    CSapientService& operator=(const CSapientService&) = delete;
    
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif /* __SAPIENT_SERVICE_H__ */

