/* sapient_tcp.h
 * 可扩展的 Sapient TCP 客户端 C API（底层用 C++ 实现，向 C 导出）
 * 目标：降低耦合、支持回调、持久连接与接收功能，兼容原有一次性发送接口。
 */
#ifndef SAPIENT_TCP_H
#define SAPIENT_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "../../inc/drone_info.h"
#include "../../common/nanopb/radar.pb.h"

/* 不透明的客户端句柄 */
typedef struct sapient_tcp_client_t sapient_tcp_client_t;

/* 回调原型：当收到数据时由调用者提供的函数将被调用（同步或异步取决于实现）。
 * data/len 指向收到的原始字节（没有长度前缀），user 为用户传入的上下文指针。
 */
typedef void (*sapient_tcp_on_message_cb)(const char *data, size_t len, void *user);

/* 创建客户端实例；host/port 可为 NULL/0，表示在连接/发送时从环境或配置回退读取。
 * 返回非 NULL 表示成功创建（但不一定已连接）。
 */
sapient_tcp_client_t *sapient_tcp_client_create(const char *host, int port);

/* 设置收到消息时的回调 */
void sapient_tcp_client_set_on_message(sapient_tcp_client_t *c, sapient_tcp_on_message_cb cb, void *user);

/* 连接到服务器（带超时，秒），0 表示使用默认超时（5 秒） */
int sapient_tcp_client_connect(sapient_tcp_client_t *c, int timeout_sec);

/* 发送原始字节（不会添加长度前缀）
 * 返回 0 表示成功，负值表示失败
 */
int sapient_tcp_client_send_raw(sapient_tcp_client_t *c, const void *data, size_t len);

/* 发送带 length-prefix 的 protobuf 消息（会自动加 4 字节 little-endian 前缀） */
int sapient_tcp_client_send_pb(sapient_tcp_client_t *c, const void *data, size_t len);

/* 发送注册报文（调用内部的 sapient_build_registration） */
int sapient_tcp_client_send_register(sapient_tcp_client_t *c);

/* 发送基于 RadarTrackItem 的 detection report（应用层数据，0x12 消息） */
int sapient_tcp_client_send_detection_report_from_track_item(sapient_tcp_client_t *c, const RadarTrackItem *track_item);

/* 发送 status report（调用内部的 sapient_build_status_report） */
int sapient_tcp_client_send_status_report(sapient_tcp_client_t *c);

/* 发送 alert report（调用内部的 sapient_build_alert_report）
 * 可传入描述、类型、状态；若传入 NULL 或 0 则采用默认值。
 */
int sapient_tcp_client_send_alert_report(sapient_tcp_client_t *c, const char *description, int type, int status);

/* 同步接收一次数据（阻塞或带超时），返回实际收到字节数或负值错误。
 * 若实现支持回调，回调也会被触发；此函数用于同步场景。
 */
int sapient_tcp_client_receive_once(sapient_tcp_client_t *c, void *buf, size_t buf_len, int timeout_sec);

/* 关闭连接并释放内部资源（不销毁结构体） */
void sapient_tcp_client_close(sapient_tcp_client_t *c);

/* 销毁客户端实例（释放内存）。在调用前应先关闭连接。 */
void sapient_tcp_client_destroy(sapient_tcp_client_t *c);

/* 启动后台接收线程（建议在连接成功后调用）。返回 0 表示启动成功。 */
int sapient_tcp_client_start_receive_thread(sapient_tcp_client_t *c);

/* 停止后台接收线程。 */
void sapient_tcp_client_stop_receive_thread(sapient_tcp_client_t *c);

/* 解析并处理收到的 SapientMessage 原始字节。
 * 若包含 Task，将自动构建并发送 TaskAck。
 * 成功返回 0；解析/处理错误返回 -1。
 */
int sapient_parse_and_handle_message(const char *data, size_t len, sapient_tcp_client_t *client);

/* 获取断网时间（从断网到现在的秒数）
 * 如果断网时间有效，返回从断网到现在的秒数；否则返回 -1
 */
int sapient_tcp_client_get_disconnect_elapsed_seconds(sapient_tcp_client_t *c);

/* 清除断网时间戳（用于：已经满足"断网>=120秒"的规则并完成一次状态上报后，恢复正常周期发送） */
void sapient_tcp_client_clear_disconnect_time(sapient_tcp_client_t *c);

/* 标记已收到 RegistrationAck（用于 30 秒超时检测机制） */
void sapient_mark_registration_ack_received(sapient_tcp_client_t *c);

/* 检查 Sapient 客户端是否在线（连接状态）
 * 返回 1 表示已连接，0 表示未连接
 */
int is_sapient_online(sapient_tcp_client_t *c);

#ifdef __cplusplus
}
#endif

#endif // SAPIENT_TCP_H
