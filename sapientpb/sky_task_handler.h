/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * All rights reserved. Any unauthorized disclosure or publication of the
 * confidential and proprietary information to any other party will constitute
 * an infringement of copyright laws.
 *
 * @file    sky_task_handler.h
 * @brief   SAPIENT Task 消息处理器
 * @details 解析 DMM 发送的 Task 消息，根据任务类型执行相应动作，
 *          并构建 TaskAck 应答消息。
 * @version 1.0
 * @date    2025-11-27
 *****************************************************************************
 */
#ifndef __SKY_TASK_HANDLER_H_
#define __SKY_TASK_HANDLER_H_

// 仅提供 C++ 接口：使用 std::string，避免 extern "C" 冲突。
#include <stddef.h>
#include <string>

// Task 请求类型枚举（用于指示需要执行的响应动作）
enum TaskActionType {
    TASK_ACTION_NONE = 0,              // 无特殊动作
    TASK_ACTION_SEND_REGISTRATION = 1, // 发送注册报文
    TASK_ACTION_SEND_STATUS = 2,       // 发送状态报文
};

/**
 * @brief 获取当前活跃的任务 ID
 * @return 当前任务 ID；若无任务则返回空字符串
 */
std::string sapient_get_current_task_id();

/**
 * @brief 设置当前活跃的任务 ID（当接受新任务时调用）
 * @param task_id 任务 ID；传入空字符串或 "0" 表示清除任务
 */
void sapient_set_current_task_id(const std::string &task_id);

/**
 * @brief 清除当前任务 ID (用于一次性任务完成后)
 */
void sapient_clear_current_task_id();

/**
 * @brief 处理 SAPIENT Task 消息并构建 TaskAck 应答
 * 
 * @param[in]  task_data          Task 的原始 protobuf 字节内容
 * @param[in]  task_len           Task 消息长度
 * @param[out] out_ack_serialized 输出封装 TaskAck 的 SapientMessage 二进制
 * @param[out] out_ack_json       输出用于调试的 JSON 文本
 * @param[out] out_action         输出需要执行的动作类型（见 TaskActionType）
 * 
 * @return int 错误码
 *         - 0: 成功
 *         - -1: 失败（解析错误或构建 TaskAck 失败）
 * 
 * @note 支持的 Task 类型：
 *       - command.request="Registration" → action=TASK_ACTION_SEND_REGISTRATION
 *       - command.request="Status" → action=TASK_ACTION_SEND_STATUS
 *       - 其他 → action=TASK_ACTION_NONE（仅回复 TaskAck）
 * 
 * @warning 该函数使用 C++ 接口（std::string），仅供 C++ 代码调用
 */
int sapient_handle_task(const void *task_data, size_t task_len,
                        std::string &out_ack_serialized, std::string &out_ack_json,
                        int &out_action);

#endif /* __SKY_TASK_HANDLER_H_ */
