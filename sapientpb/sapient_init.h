/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * All rights reserved. Any unauthorized disclosure or publication of the
 * confidential and proprietary information to any other party will constitute
 * an infringement of copyright laws.
 *
 * @file    sapient_init.h
 * @brief   SAPIENT (BSI Flex 335 v2.0) 协议模块初始化接口
 * @details 提供 SAPIENT 协议客户端的生命周期管理，包括创建、连接、
 *          消息收发和线程管理。支持 DMM (Data & Message Manager) 对接。
 * @version 1.0
 * @date    2025-11-27
 * 
 * @note    线程安全性说明：
 *          - sapient_init() 不可重入，应在 main 线程中调用一次
 *          - get_sapient_client() 返回的句柄全局唯一，多线程读安全
 *          - 发送接口内部有锁保护，可在多线程中安全调用
 *          - 接收由后台线程处理，回调在接收线程上下文执行
 *****************************************************************************
 */
#ifndef __SAPIENT_INIT_H_
#define __SAPIENT_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sapient_tcp.h"

/* SAPIENT 模块错误码定义 */
typedef enum {
    SAPIENT_OK = 0,                      /* 成功 */
    SAPIENT_ERR_NOT_CONFIGURED = -1,     /* 配置未设置 */
    SAPIENT_ERR_CREATE_FAILED = -2,      /* 创建客户端失败 */
    SAPIENT_ERR_CONNECT_FAILED = -3,     /* 连接服务器失败 */
    SAPIENT_ERR_THREAD_FAILED = -4,      /* 启动接收线程失败 */
} sapient_error_t;

/**
 * @brief SAPIENT 模块初始化
 * 
 * @details 从 dev_config 读取配置，创建 TCP 客户端，连接 DMM 服务器，
 *          发送注册报文，启动后台接收线程。
 *          该函数应在 main 线程中调用一次，不可重复调用。
 * 
 * @return sapient_error_t 错误码
 *         - SAPIENT_OK: 初始化成功
 *         - SAPIENT_ERR_NOT_CONFIGURED: sapient.ip/port 未配置
 *         - SAPIENT_ERR_CREATE_FAILED: 创建客户端失败
 *         - SAPIENT_ERR_CONNECT_FAILED: 连接 DMM 失败
 *         - SAPIENT_ERR_THREAD_FAILED: 启动接收线程失败
 * 
 * @note 连接成功后会自动发送 Registration 报文
 * @warning 非线程安全，仅允许调用一次
 */
int sapient_init(void);

/**
 * @brief SAPIENT 模块清理
 * 
 * @details 停止接收线程，关闭 TCP 连接，释放资源。
 *          可在程序退出时调用，非必须。
 * 
 * @warning 调用后 get_sapient_client() 将返回 NULL
 */
void sapient_cleanup(void);

/**
 * @brief 获取全局 SAPIENT TCP 客户端句柄
 * 
 * @return sapient_tcp_client_t* 客户端句柄
 *         - 非 NULL: 有效句柄，可用于发送消息
 *         - NULL: 未初始化或已清理
 * 
 * @note 线程安全：多线程可并发读取，返回的句柄不可修改
 * @note 返回的句柄由 sapient_init 管理，调用者不应释放
 */
sapient_tcp_client_t *get_sapient_client(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAPIENT_INIT_H_ */
