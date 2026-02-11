#include "../sapient/task.pb.h"
#include "../sapient/task_ack.pb.h"
#include "../sapient/sapient_message.pb.h"
#include "sky_task_handler.h"
#include "sapient_nodeid.h"
#include <iostream>
#include <chrono>
#include <string>
#include <algorithm>
#include <cctype>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/timestamp.pb.h>
#include <mutex>

// 定义日志模块标签
#define LOG_TAG "sapient_task"
extern "C" {
    #include "../../common/zlog/skyfend_log.h"
}

// 兼容宏：将 LOGI/LOGE 映射到 radar_log_info/radar_log_error
#define LOGI(format, ...) radar_log_info(format, ##__VA_ARGS__)
#define LOGE(format, ...) radar_log_error(format, ##__VA_ARGS__)

// External references
// extern std::string g_nodeId; // Replaced by sapient_nodeid.h

// 静态变量：存储当前活跃的任务ID
static std::string s_current_task_id;
static std::mutex s_task_id_mutex;

// 工具函数：不区分大小写字符串比较
static bool iequals(const std::string &a, const std::string &b) {
    return a.size() == b.size() && 
           std::equal(a.begin(), a.end(), b.begin(),
                      [](char ca, char cb) { return std::tolower(ca) == std::tolower(cb); });
}

// 工具函数：为消息封装设置当前 UTC 时间戳
static void set_current_timestamp(::google::protobuf::Timestamp *ts)
{
    auto now = std::chrono::system_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
    ts->set_seconds(seconds);
    ts->set_nanos((int32_t)nanos);
}

// 内部处理：解析接收到的 Task 并决定是否接受
// 返回 true 表示接受，false 表示拒绝；
// 同时在 reason_out 中填入原因说明，action_out 中返回需要执行的动作类型。
static bool handle_task_internal(const sapient_msg::bsi_flex_335_v2_0::Task &task, 
                                  std::string &reason_out, TaskActionType &action_out)
{
    using namespace sapient_msg::bsi_flex_335_v2_0;
    
    action_out = TASK_ACTION_NONE;
    
    // 打印收到的任务 ID 与控制指令
    std::string task_id = task.has_task_id() ? task.task_id() : "(no task_id)";
    LOGI("received Sapient Task: task_id=%s\n", task_id.c_str());

    // 提取顶层控制指令（control 字段）
    if (task.has_control()) {
        Task::Control ctrl = task.control();
        const char *ctrl_str = "UNKNOWN";
        switch (ctrl) {
            case Task::CONTROL_START:  ctrl_str = "START";  break;
            case Task::CONTROL_STOP:   ctrl_str = "STOP";   break;
            case Task::CONTROL_PAUSE:  ctrl_str = "PAUSE";  break;
            default: break;
        }
        LOGI("  Task control=%s\n", ctrl_str);
    }

    // 提取 command.request 字段（关键：判断任务类型）
    if (task.has_command() && task.command().has_request()) {
        std::string request = task.command().request();
        LOGI("  Task command.request=%s\n", request.c_str());
        
        // 根据请求类型设置响应动作（支持不区分大小写匹配）
        if (iequals(request, "Registration") || iequals(request, "Request Registration")) {
            action_out = TASK_ACTION_SEND_REGISTRATION;
            reason_out = "Task accepted, will send Registration report";
        } else if (iequals(request, "Status") || iequals(request, "Request Status")) {
            action_out = TASK_ACTION_SEND_STATUS;
            reason_out = "Task accepted, will send Status report";
        } else {
            reason_out = "Task accepted, unknown request type: " + request;
        }
    } else {
        reason_out = "Task accepted for processing";
    }

    // 提取区域（如存在）
    if (task.region_size() > 0) {
        LOGI("  Task region count=%d\n", task.region_size());
    }

    // 目前：统一接受所有任务
    // TODO：实现真实任务执行与校验（启动/停止检测、应用筛选器等）
    return true;
}

// 构建 TaskAck 响应，并封装到 SapientMessage。
// 成功返回 0，失败返回 -1。
// out_serialized：封装 TaskAck 的 SapientMessage 二进制
// out_json：便于调试的 JSON 文本
int sapient_build_task_ack(std::string &out_serialized, std::string &out_json,
                            const std::string &task_id_in, bool accepted, const std::string &reason_in)
{
    using namespace sapient_msg::bsi_flex_335_v2_0;

    // Build TaskAck message
    TaskAck ack;
    if (!task_id_in.empty()) {
        ack.set_task_id(task_id_in);
    }
    if (accepted) {
        ack.set_task_status(TaskAck::TASK_STATUS_ACCEPTED);
    } else {
        ack.set_task_status(TaskAck::TASK_STATUS_REJECTED);
    }
    if (!reason_in.empty()) {
        ack.add_reason(reason_in);
    }

    // 将 TaskAck 封装到 SapientMessage 中
    SapientMessage wrapper;
    set_current_timestamp(wrapper.mutable_timestamp());
    std::string node_id = generateNodeID();
    if (!node_id.empty()) {
        wrapper.set_node_id(node_id);
    }
    wrapper.set_allocated_task_ack(new TaskAck(ack));

    // 序列化为二进制
    if (!wrapper.SerializeToString(&out_serialized)) {
        std::cerr << "Failed to serialize TaskAck message" << std::endl;
        return -1;
    }

    // 转换为 JSON（用于日志打印）
    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    options.always_print_primitive_fields = true;
    if (!google::protobuf::util::MessageToJsonString(wrapper, &out_json, options).ok()) {
        std::cerr << "Failed to convert TaskAck message to JSON" << std::endl;
        return -1;
    }

    return 0;
}

// 处理 Task 原始字节（来源于 SapientMessage.task）
// 构建并返回 TaskAck 响应（使用 C++ 链接，因为涉及 std::string）。
int sapient_handle_task(const void *task_data, size_t task_len, 
                        std::string &out_ack_serialized, std::string &out_ack_json,
                        int &out_action)
{
    using namespace sapient_msg::bsi_flex_335_v2_0;

    // 从字节解析 Task
    Task task;
    if (!task.ParseFromArray(task_data, (int)task_len)) {
        LOGE("Failed to parse Task message\n");
        return -1;
    }

    // 处理任务并决定接受/拒绝，同时获取需要执行的动作
    std::string reason;
    TaskActionType action = TASK_ACTION_NONE;
    bool accepted = handle_task_internal(task, reason, action);

    out_action = (int)action;

    // 构建 TaskAck 响应
    std::string task_id = task.has_task_id() ? task.task_id() : "";
    
    // 如果任务被接受，设置当前任务ID（用于在status report中显示）
    if (accepted && !task_id.empty()) {
        sapient_set_current_task_id(task_id);
    }
    
    int ret = sapient_build_task_ack(out_ack_serialized, out_ack_json, task_id, accepted, reason);
    if (ret != 0) {
        LOGE("sapient_build_task_ack failed\n");
        return -1;
    }

    LOGI("Task handled; TaskAck prepared (accepted=%d, action=%d)\n", accepted, out_action);
    return 0;
}

// 获取当前活跃的任务ID（线程安全）
// 返回当前任务ID，若无活跃任务则返回空字符串
std::string sapient_get_current_task_id() {
    std::lock_guard<std::mutex> lock(s_task_id_mutex);
    return s_current_task_id;
}

// 设置当前活跃的任务ID（线程安全）
// task_id 为空字符串或 "0" 时，等同于清除任务ID
void sapient_set_current_task_id(const std::string &task_id) {
    std::lock_guard<std::mutex> lock(s_task_id_mutex);
    if (task_id.empty() || task_id == "0") {
        s_current_task_id.clear();
    } else {
        s_current_task_id = task_id;
    }
}

// 清除当前活跃的任务ID（线程安全）
// 用于一次性任务（如Registration、Status、Detection）执行完成后清除
void sapient_clear_current_task_id() {
    std::lock_guard<std::mutex> lock(s_task_id_mutex);
    s_current_task_id.clear();
}
