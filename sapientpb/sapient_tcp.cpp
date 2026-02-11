#include "sapient_tcp.h"
#include "../../common/nanopb/radar.pb.h"
#include "../sapient/sapient_message.pb.h"
#include "../sapient/task.pb.h"
#include "sky_task_handler.h"
#include <string>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>

// socket 相关头文件
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>

// 日志模块
#define LOG_TAG "sapient_tcp"
extern "C" {
    #include "../../common/zlog/skyfend_log.h"
}

// 兼容宏：将 LOGI/LOGE 映射到 radar_log_info/radar_log_error
#define LOGI(format, ...) radar_log_info(format, ##__VA_ARGS__)
#define LOGE(format, ...) radar_log_error(format, ##__VA_ARGS__)

// 从 sky_registrationpb.cpp 中声明的构建函数
int sapient_build_registration(std::string &out_serialized, std::string &out_json);
// 基于 RadarTrackItem 的 detection report 构建函数（C++ 接口）
extern "C" int sapient_build_detection_report_from_track_item_cpp(std::string &out_serialized, std::string &out_json, const RadarTrackItem *track_item);
// 从 sky_status_reportpb.cpp 中声明的构建函数
int sapient_build_status_report(std::string &out_serialized, std::string &out_json);
// 从 sky_alert_reportpb.cpp 中声明的构建函数
int sapient_build_alert_report(std::string &out_serialized, std::string &out_json,
                               const char *description, int type, int status);

// 简单的 C++ 封装类，提供连接、发送、接收、回调功能。
class SapientTcpClientImpl {
public:
    SapientTcpClientImpl(const std::string &h, int p)
        : host(h), port(p), sockfd(-1), on_msg(nullptr), user(nullptr), running(false), is_connected(false),
          disconnect_time_valid_(false), registration_ack_received_(false), waiting_for_registration_ack_(false) {}

    // 注意：send_mutex_ 保护所有 TCP 发送操作（send_pb / send_all），
    // 确保 4 字节长度前缀 + 消息体作为原子操作写入 socket。
    // 否则多线程（状态报告线程 + 跟踪数据线程）并发发送时，
    // 字节流会交错，导致 SDA 帧解析器失步，所有后续消息不可解析。

    ~SapientTcpClientImpl() {
        stop_receive_thread();
        close_socket();
    }

    // 尝试连接（带超时）
    int connect_with_timeout(int timeout_sec) {
        const char *use_host = host.empty() ? getenv("SAPIENT_HOST") : host.c_str();
        int use_port = (port <= 0) ? (getenv("SAPIENT_PORT") ? atoi(getenv("SAPIENT_PORT")) : 0) : port;
        if (!use_host || use_port <= 0) {
            LOGE("sapient client: invalid host/port\n");
            return -1;
        }

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            std::cerr << "socket() failed: " << strerror(errno) << std::endl;
            return -1;
        }

        struct sockaddr_in srv;
        memset(&srv, 0, sizeof(srv));
        srv.sin_family = AF_INET;
        srv.sin_port = htons(use_port);
        if (inet_pton(AF_INET, use_host, &srv.sin_addr) <= 0) {
            std::cerr << "inet_pton failed for host " << use_host << std::endl;
            close(sockfd); sockfd = -1; return -1;
        }

        int flags = fcntl(sockfd, F_GETFL, 0);
        if (flags >= 0) fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

        int rc = connect(sockfd, (struct sockaddr*)&srv, sizeof(srv));
        if (rc < 0 && errno != EINPROGRESS) {
            std::cerr << "connect() failed: " << strerror(errno) << std::endl;
            close(sockfd); sockfd = -1; return -1;
        }

        if (rc < 0) {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(sockfd, &wfds);
            struct timeval tv;
            tv.tv_sec = (timeout_sec > 0 ? timeout_sec : 5);
            tv.tv_usec = 0;
            rc = select(sockfd + 1, NULL, &wfds, NULL, &tv);
            if (rc <= 0) {
                std::cerr << "connect timeout or select error" << std::endl;
                close(sockfd); sockfd = -1; return -1;
            }
            int so_error = 0; socklen_t len = sizeof(so_error);
            getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &so_error, &len);
            if (so_error != 0) {
                std::cerr << "socket error after select: " << strerror(so_error) << std::endl;
                close(sockfd); sockfd = -1; return -1;
            }
        }

        // 设置 TCP_NODELAY（禁用 Nagle 算法，降低延迟）
        int nodelay = 1;
        if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)) < 0) {
            std::cerr << "setsockopt(TCP_NODELAY) failed: " << strerror(errno) << std::endl;
        }
        
        // 设置 SO_KEEPALIVE（启用 TCP keepalive，快速检测断开）
        int keepalive = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
            std::cerr << "setsockopt(SO_KEEPALIVE) failed: " << strerror(errno) << std::endl;
        }
        
        // 设置 keepalive 参数：10秒开始探测，5秒间隔，3次失败即断开（总共约20秒）
        int keepidle = 10;   // 10秒空闲后开始探测
        int keepintvl = 5;   // 每5秒探测一次
        int keepcnt = 3;     // 3次失败即认为断开
        setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
        setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
        setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));

        // 恢复阻塞
        if (flags >= 0) fcntl(sockfd, F_SETFL, flags);
        
        is_connected = true;  // 连接成功，标记为已连接
        // 注意：不要在这里清除 disconnect_time_valid_。
        // 断线时间戳用于“断网后 2 分钟规则”（registration / status 发送策略），
        // 应由上层在满足条件并完成一次动作后主动清除。
        return 0;
    }

    void close_socket() {
        if (sockfd >= 0) { close(sockfd); sockfd = -1; }
        is_connected = false;  // 标记连接已断开
        // 记录断线时间戳（用于判断重连时是否需要发送 registration）
        // 如果已经有时间戳，不更新（保留更早的断线时间，更符合规范要求）
        if (!disconnect_time_valid_) {
            disconnect_time_ = std::chrono::steady_clock::now();
            disconnect_time_valid_ = true;
        }
    }

    // 内部发送函数，支持可选的锁参数（避免死锁）
    int send_all_impl(const void *data, size_t len, bool already_locked = false) {
        // 发送前检查连接状态（使用 flag 快速检查，避免频繁系统调用）
        if (sockfd < 0 || !is_connected.load()) {
            std::unique_lock<std::mutex> lock(reconnect_mutex, std::defer_lock);
            if (!already_locked) {
                lock.lock();
            }
            // 双重检查：可能在获取锁的过程中，其他线程已经重连成功
            // 如果此时 is_connected 已为 true，说明其他线程已重连，直接跳过
            if (sockfd < 0 || !is_connected.load()) {
                // 如果 sockfd >= 0 但 is_connected 为 false，可能是误判，再确认一次
                if (sockfd >= 0 && is_socket_alive()) {
                    is_connected = true;  // 连接实际正常，更新 flag
                } else {
                    // 确实断开，尝试重连
                    LOGI("Socket disconnected, attempting reconnect before send\n");
                    if (reconnect_with_backoff() != 0) {
                        return -1;  // 重连失败
                    }
                    // 重连成功，已在 reconnect_with_backoff() 中发送注册报文
                }
            }
        }
        
        const uint8_t *p = (const uint8_t*)data;
        size_t remaining = len;
        while (remaining > 0) {
            ssize_t n = send(sockfd, p, remaining, 0);
            if (n <= 0) {
                // 发送失败，可能是连接断开
                if (errno == EPIPE || errno == ECONNRESET || errno == ENOTCONN) {
                    is_connected = false;  // 标记连接已断开
                    // 记录断线时间
                    if (!disconnect_time_valid_) {
                        disconnect_time_ = std::chrono::steady_clock::now();
                        disconnect_time_valid_ = true;
                    }
                    LOGI("Send failed due to connection error, attempting reconnect\n");
                    std::unique_lock<std::mutex> lock(reconnect_mutex, std::defer_lock);
                    if (!already_locked) {
                        lock.lock();
                    }
                    if (reconnect_with_backoff() == 0) {
                        // 重连成功，是否发送注册报文由 reconnect_with_backoff() 根据断线时间决定
                        // 继续发送剩余数据（极小概率部分发送场景，实际中几乎不会发生）
                        continue;
                    }
                }
                LOGE("send() failed: %s\n", strerror(errno));
                return -1;
            }
            remaining -= (size_t)n; p += n;
        }
        return 0;
    }

    // 公开接口：发送数据（自动处理重连）
    int send_all(const void *data, size_t len) {
        return send_all_impl(data, len, false);
    }

    // 内部发送protobuf函数，支持可选的锁参数（避免死锁）
    // 关键修复：使用 send_mutex_ 保护整个 send_pb 操作（4字节长度前缀 + 消息体）
    // 防止多线程（状态报告线程 + 跟踪数据线程）并发发送导致字节流交错
    int send_pb_impl(const void *data, size_t len, bool already_locked = false) {
        std::lock_guard<std::mutex> send_lock(send_mutex_);  // 原子化保护
        // 4 bytes little-endian prefix
        uint32_t body_len = (uint32_t)len;
        uint8_t len_buf[4];
        len_buf[0] = (uint8_t)(body_len & 0xFF);
        len_buf[1] = (uint8_t)((body_len >> 8) & 0xFF);
        len_buf[2] = (uint8_t)((body_len >> 16) & 0xFF);
        len_buf[3] = (uint8_t)((body_len >> 24) & 0xFF);
        if (send_all_impl(len_buf, sizeof(len_buf), already_locked) != 0) return -1;
        return send_all_impl(data, len, already_locked);
    }

    // 公开接口：发送protobuf数据
    int send_pb(const void *data, size_t len) {
        return send_pb_impl(data, len, false);
    }

    int send_register() {
        std::string bin, json;
        if (sapient_build_registration(bin, json) != 0) {
            std::cerr << "sapient_build_registration failed" << std::endl;
            return -1;
        }
        
        // 记录 Registration 发送时间，启动 30 秒超时检测
        {
            std::lock_guard<std::mutex> lock(registration_mutex_);
            registration_sent_time_ = std::chrono::steady_clock::now();
            registration_ack_received_ = false;
            waiting_for_registration_ack_ = true;
        }
        LOGI("Registration sent, waiting for RegistrationAck (30 second timeout)\n");
        
        return send_pb(bin.data(), bin.size());
    }


    // 发送基于 RadarTrackItem 的 detection report（应用层数据，0x12 消息）
    int send_detection_report_from_track_item(const RadarTrackItem *track_item) {
        if (!track_item) {
            LOGE("send_detection_report_from_track_item: track_item is null\n");
            return -1;
        }
        
        std::string bin, json;
        if (sapient_build_detection_report_from_track_item_cpp(bin, json, track_item) != 0) {
            LOGE("sapient_build_detection_report_from_track_item failed\n");
            return -1;
        }
        
        return send_pb(bin.data(), bin.size());
    }

    // 发送 status report
    int send_status_report() {
        std::string bin, json;
        if (sapient_build_status_report(bin, json) != 0) {
            std::cerr << "sapient_build_status_report failed" << std::endl;
            return -1;
        }
        return send_pb(bin.data(), bin.size());
    }

    // 同步接收一次：解析 4 字节小端长度前缀，随后读取完整消息体；
    // 触发回调（如已设置），并将消息体拷贝到调用者缓冲区（若提供且有空间）。
    // 返回值：>0 为拷贝到 buf 的消息体字节数；0 为超时；负值为错误。
    int receive_once(void *buf, size_t buf_len, int timeout_sec) {
        if (sockfd < 0) return -1;

        auto recv_fully = [&](uint8_t *dst, size_t need) -> int {
            size_t got = 0;
            while (got < need) {
                fd_set rfds; FD_ZERO(&rfds); FD_SET(sockfd, &rfds);
                struct timeval tv; tv.tv_sec = timeout_sec; tv.tv_usec = 0;
                int rc = select(sockfd + 1, &rfds, NULL, NULL, &tv);
                if (rc == 0) return 0; // 超时
                if (rc < 0) return -1; // select 错误
                ssize_t n = recv(sockfd, dst + got, need - got, 0);
                if (n <= 0) return -1; // 连接关闭或错误
                got += (size_t)n;
            }
            return (int)got;
        };

        // 读取 4 字节长度前缀（小端）
        uint8_t len_buf[4];
        int hn = recv_fully(len_buf, sizeof(len_buf));
        if (hn <= 0) return hn; // 0=超时，-1=错误
        uint32_t body_len = (uint32_t)len_buf[0]
                          | ((uint32_t)len_buf[1] << 8)
                          | ((uint32_t)len_buf[2] << 16)
                          | ((uint32_t)len_buf[3] << 24);
        if (body_len == 0 || body_len > (32u * 1024u * 1024u)) {
            std::cerr << "invalid sapient frame length: " << body_len << std::endl;
            return -1;
        }

        // 读取完整消息体
        std::string body;
        body.resize(body_len);
        int bn = recv_fully(reinterpret_cast<uint8_t*>(&body[0]), body_len);
        if (bn <= 0) return bn; // 0=超时，-1=错误

        // 回调传入完整消息体（不含前缀）
        if (on_msg) on_msg(body.data(), body.size(), user);

        // 如果调用者提供缓冲区，则拷贝消息体
        int copy_len = (int)std::min(buf_len, (size_t)body_len);
        if (buf && copy_len > 0) {
            memcpy(buf, body.data(), (size_t)copy_len);
        }
        return copy_len;
    }

    void set_on_message(sapient_tcp_on_message_cb cb, void *u) { on_msg = cb; user = u; }

    // 启动后台接收线程：循环读取完整帧并触发回调
    // 接收线程负责检测连接断开并自动重连
    int start_receive_thread() {
        if (running) {
            LOGI("Receive thread already running\n");
            return 0; // 已启动
        }
        if (sockfd < 0) {
            LOGE("Cannot start receive thread: socket not connected\n");
            return -1;
        }
        LOGI("Starting receive thread...\n");
        running = true;
        recv_thread = std::thread([this]() {
            LOGI("Receive thread started successfully\n");
            std::vector<uint8_t> tmp(64 * 1024);
            int consecutive_errors = 0;  // 连续错误计数
            const int max_consecutive_errors = 3;  // 连续3次错误才认为断开
            
            while (running) {
                // ========== 检查 RegistrationAck 30 秒超时 ==========
                {
                    std::lock_guard<std::mutex> lock(registration_mutex_);
                    if (waiting_for_registration_ack_) {
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            now - registration_sent_time_).count();
                        
                        // 根据 Sapient 规范：30 秒内未收到 RegistrationAck，必须重连并重发
                        if (elapsed >= 30) {
                            LOGE("RegistrationAck timeout (%ld seconds), triggering reconnect per Sapient spec\n", 
                                 elapsed);
                            waiting_for_registration_ack_ = false;
                            is_connected = false;  // 标记连接断开
                            
                            // 强制重连并重发 Registration
                            std::lock_guard<std::mutex> rlock(reconnect_mutex);
                            LOGI("Reconnecting due to RegistrationAck timeout...\n");
                            if (reconnect_with_backoff(true) == 0) {  // true = 强制发送 registration
                                LOGI("Reconnected successfully after RegistrationAck timeout\n");
                                consecutive_errors = 0;
                            } else {
                                LOGE("Reconnect failed after RegistrationAck timeout\n");
                                std::this_thread::sleep_for(std::chrono::seconds(5));
                            }
                            continue;  // 跳过本次接收，直接进入下一轮循环
                        }
                    }
                }
                // ===================================================
                
                int ret = this->receive_once(tmp.data(), tmp.size(), 1);
                if (ret < 0) {
                    consecutive_errors++;
                    // 连续多次错误才认为连接真正断开（避免网络抖动误判）
                    if (consecutive_errors >= max_consecutive_errors) {
                        is_connected = false;  // 标记连接已断开
                        // 记录断线时间（在 close_socket() 中也会记录，但这里提前记录更准确）
                        if (!disconnect_time_valid_) {
                            disconnect_time_ = std::chrono::steady_clock::now();
                            disconnect_time_valid_ = true;
                        }
                        LOGI("Connection lost detected, attempting reconnect\n");
                        std::lock_guard<std::mutex> lock(reconnect_mutex);
                        LOGI("Calling reconnect_with_backoff() from receive thread\n");
                        if (reconnect_with_backoff() == 0) {
                            LOGI("Reconnected successfully\n");
                            consecutive_errors = 0;  // 重置错误计数
                            // 重连成功，是否发送注册报文由 reconnect_with_backoff() 根据断线时间决定
                        } else {
                            LOGE("Reconnect failed in receive thread\n");
                            // 重连失败，等待后继续尝试
                            std::this_thread::sleep_for(std::chrono::seconds(5));
                        }
                    } else {
                        // 可能是临时错误，短暂等待后继续
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                } else if (ret == 0) {
                    // 超时是正常情况，重置错误计数
                    consecutive_errors = 0;
                    continue;
                } else {
                    // 成功接收，重置错误计数
                    consecutive_errors = 0;
                    // 回调已在 receive_once 内部触发
                }
            }
        });
        return 0;
    }

    // 停止后台接收线程
    void stop_receive_thread() {
        if (!running) return;
        running = false;
        if (recv_thread.joinable()) recv_thread.join();
    }

    // 自动重连机制：尝试重新建立连接，并在重连成功后根据断线时间决定是否发送注册报文
    // 根据 Sapient 规范：每 10 秒尝试连接一次，直到成功
    // 根据 Sapient 规范：如果重连发生在断线后2分钟内，不需要重新发送 registration message
    // 参数 send_registration: 是否强制发送注册报文（默认false，由断线时间自动决定）
    int reconnect_with_backoff(bool force_send_registration = false) {
        const int reconnect_interval_seconds = 10;  // Sapient 规范要求：每 10 秒尝试一次
        
        close_socket();  // 先关闭旧socket，释放资源
        
        int attempt = 0;
        while (running) {
            attempt++;
            LOGI("Reconnecting attempt %d (interval: %d seconds, per Sapient spec)\n", 
                 attempt, reconnect_interval_seconds);
            
            // 尝试连接（超时5秒）
            int ret = connect_with_timeout(5);
            if (ret == 0) {
                LOGI("Reconnection successful after %d attempts\n", attempt);
                is_connected = true;  // 重连成功，标记为已连接
                
                // 根据 Sapient 规范判断是否需要发送 registration message：
                // 如果重连发生在断线后2分钟内，不需要重新发送 registration message
                bool need_send_registration = force_send_registration;
                if (!need_send_registration && disconnect_time_valid_) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - disconnect_time_).count();
                    const int64_t registration_timeout_seconds = 120;  // 2分钟 = 120秒
                    
                    if (elapsed >= registration_timeout_seconds) {
                        need_send_registration = true;
                        LOGI("Disconnection time exceeded %ld seconds (%ld seconds elapsed), registration required\n",
                             registration_timeout_seconds, elapsed);
                    } else {
                        LOGI("Reconnection within %ld seconds (%ld seconds elapsed), registration not required\n",
                             registration_timeout_seconds, elapsed);
                    }
                } else if (!disconnect_time_valid_) {
                    // 首次连接或断线时间戳无效，需要发送 registration
                    need_send_registration = true;
                    LOGI("First connection or invalid disconnect time, registration required\n");
                }
                
                // 重连成功后根据判断结果决定是否发送注册报文
                LOGI("Reconnect successful, need_send_registration=%d\n", need_send_registration);
                if (need_send_registration) {
                    LOGI("Sending registration after reconnection\n");
                    std::string bin, json;
                    if (sapient_build_registration(bin, json) == 0) {
                        // 注意：这里不锁 send_mutex_，因为 reconnect_with_backoff() 可能从
                        // send_all_impl() 调用，而 send_all_impl() 的调用者 send_pb_impl()
                        // 已经持有 send_mutex_，加锁会死锁。
                        // 此处安全性保证：新 socket 刚创建，其他线程被 reconnect_mutex 阻塞，
                        // 不可能同时在新 socket 上发送。
                        
                        // 先发送4字节长度前缀
                        uint32_t body_len = (uint32_t)bin.size();
                        uint8_t len_buf[4];
                        len_buf[0] = (uint8_t)(body_len & 0xFF);
                        len_buf[1] = (uint8_t)((body_len >> 8) & 0xFF);
                        len_buf[2] = (uint8_t)((body_len >> 16) & 0xFF);
                        len_buf[3] = (uint8_t)((body_len >> 24) & 0xFF);
                        
                        // 完整发送长度前缀（循环发送确保完整）
                        size_t sent = 0;
                        while (sent < sizeof(len_buf)) {
                            ssize_t n = ::send(sockfd, len_buf + sent, sizeof(len_buf) - sent, 0);
                            if (n <= 0) {
                                LOGE("Failed to send registration length prefix: %s\n", strerror(errno));
                                break;
                            }
                            sent += n;
                        }
                        
                        // 完整发送消息体（循环发送确保完整）
                        if (sent == sizeof(len_buf)) {
                            sent = 0;
                            while (sent < bin.size()) {
                                ssize_t n = ::send(sockfd, bin.data() + sent, bin.size() - sent, 0);
                                if (n <= 0) {
                                    LOGE("Failed to send registration body: %s\n", strerror(errno));
                                    break;
                                }
                                sent += n;
                            }
                            if (sent == bin.size()) {
                                LOGI("Registration sent successfully (%zu bytes)\n", bin.size());
                            }
                        }
                    } else {
                        LOGE("Failed to build registration message\n");
                    }
                }
                
                // 注意：这里也不要清除 disconnect_time_valid_。
                // 断线时间戳需要保留给“断网后 2 分钟规则”的上层逻辑使用。
                
                return 0;  // 重连成功
            }
            
            // 根据 Sapient 规范：每 10 秒尝试一次，直到成功
            // 等待 10 秒后继续下一次尝试
            std::this_thread::sleep_for(std::chrono::seconds(reconnect_interval_seconds));
        }
        
        // 如果 running 变为 false，说明程序正在退出
        LOGI("Reconnection stopped (running flag set to false)\n");
        return -1;
    }

    // 检查socket是否有效（通过发送0字节数据检测）
    bool is_socket_alive() {
        if (sockfd < 0) return false;
        
        // 使用MSG_DONTWAIT和MSG_NOSIGNAL标志，避免阻塞和信号
        char buf;
        ssize_t n = recv(sockfd, &buf, 1, MSG_PEEK | MSG_DONTWAIT);
        
        if (n == 0) {
            // 对端关闭连接
            return false;
        } else if (n < 0) {
            // 检查是否是"暂时无数据"错误（正常情况）
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return true;  // socket正常，只是暂时没数据
            }
            // 其他错误表示连接已断开
            return false;
        }
        return true;  // 有数据可读，socket正常
    }

    // 获取断网时间（从断网到现在的秒数）
    int get_disconnect_elapsed_seconds() {
        if (!disconnect_time_valid_) return -1;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - disconnect_time_).count();
        return (int)elapsed;
    }

    void clear_disconnect_time() { disconnect_time_valid_ = false; }

    // 检查是否在线（连接状态）
    bool is_online() const {
        return is_connected.load();
    }

    // 标记收到 RegistrationAck（由外部消息解析器调用）
    void mark_registration_ack_received() {
        std::lock_guard<std::mutex> lock(registration_mutex_);
        if (waiting_for_registration_ack_) {
            registration_ack_received_ = true;
            waiting_for_registration_ack_ = false;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - registration_sent_time_).count();
            LOGI("RegistrationAck received after %ld ms\n", elapsed);
        }
    }

private:
    std::string host;
    int port;
    int sockfd;
    sapient_tcp_on_message_cb on_msg;
    void *user;
    std::thread recv_thread;
    std::atomic<bool> running;
    std::atomic<bool> is_connected;  // 连接状态标志（避免频繁调用 is_socket_alive()）
    std::mutex reconnect_mutex;  // 保护重连过程，避免并发重连
    std::mutex send_mutex_;  // 保护 TCP 发送操作，确保 length-prefix + body 原子写入
    std::chrono::steady_clock::time_point disconnect_time_;  // 记录断线时间戳
    bool disconnect_time_valid_;  // 断线时间戳是否有效
    
    // RegistrationAck 超时检测机制（30秒超时，根据 Sapient 规范）
    std::chrono::steady_clock::time_point registration_sent_time_;  // Registration 发送时间
    std::atomic<bool> registration_ack_received_;  // 是否收到 RegistrationAck
    std::atomic<bool> waiting_for_registration_ack_;  // 是否正在等待 RegistrationAck
    std::mutex registration_mutex_;  // 保护 registration 相关状态
};

// C 包装器结构
struct sapient_tcp_client_t { SapientTcpClientImpl *impl; };

extern "C" {

sapient_tcp_client_t *sapient_tcp_client_create(const char *host, int port) {
    sapient_tcp_client_t *c = new sapient_tcp_client_t();
    std::string h = host ? host : std::string();
    c->impl = new SapientTcpClientImpl(h, port);
    return c;
}

void sapient_tcp_client_set_on_message(sapient_tcp_client_t *c, sapient_tcp_on_message_cb cb, void *user) {
    if (!c || !c->impl) return;
    c->impl->set_on_message(cb, user);
}

int sapient_tcp_client_connect(sapient_tcp_client_t *c, int timeout_sec) {
    if (!c || !c->impl) return -1;
    return c->impl->connect_with_timeout(timeout_sec);
}

int sapient_tcp_client_send_raw(sapient_tcp_client_t *c, const void *data, size_t len) {
    if (!c || !c->impl) return -1;
    return c->impl->send_all(data, len);
}

int sapient_tcp_client_send_pb(sapient_tcp_client_t *c, const void *data, size_t len) {
    if (!c || !c->impl) return -1;
    return c->impl->send_pb(data, len);
}

int sapient_tcp_client_send_register(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return -1;
    return c->impl->send_register();
}

int sapient_tcp_client_send_detection_report_from_track_item(sapient_tcp_client_t *c, const RadarTrackItem *track_item) {
    if (!c || !c->impl) return -1;
    return c->impl->send_detection_report_from_track_item(track_item);
}

int sapient_tcp_client_send_status_report(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return -1;
    return c->impl->send_status_report();
}

int sapient_tcp_client_send_alert_report(sapient_tcp_client_t *c, const char *description, int type, int status) {
    if (!c || !c->impl) return -1;
    // 直接构建并发送（最小版本）
    std::string bin, json;
    if (sapient_build_alert_report(bin, json, description, type, status) != 0) {
        LOGE("sapient_build_alert_report failed\n");
        return -1;
    }
    return c->impl->send_pb(bin.data(), bin.size());
}

int sapient_tcp_client_receive_once(sapient_tcp_client_t *c, void *buf, size_t buf_len, int timeout_sec) {
    if (!c || !c->impl) return -1;
    return c->impl->receive_once(buf, buf_len, timeout_sec);
}

void sapient_tcp_client_close(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return;
    c->impl->stop_receive_thread();
    c->impl->close_socket();
}

void sapient_tcp_client_destroy(sapient_tcp_client_t *c) {
    if (!c) return;
    if (c->impl) delete c->impl;
    delete c;
}

// 启动后台接收线程（建议在成功连接后调用）
int sapient_tcp_client_start_receive_thread(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return -1;
    return c->impl->start_receive_thread();
}

// 停止后台接收线程
void sapient_tcp_client_stop_receive_thread(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return;
    c->impl->stop_receive_thread();
}

// 获取断网时间（从断网到现在的秒数）
int sapient_tcp_client_get_disconnect_elapsed_seconds(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return -1;
    return c->impl->get_disconnect_elapsed_seconds();
}

void sapient_tcp_client_clear_disconnect_time(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return;
    c->impl->clear_disconnect_time();
}

void sapient_mark_registration_ack_received(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return;
    c->impl->mark_registration_ack_received();
}

int is_sapient_online(sapient_tcp_client_t *c) {
    if (!c || !c->impl) return 0;
    return c->impl->is_online() ? 1 : 0;
}

/* 解析收到的字节为 SapientMessage，并按内容类型进行分发。
 * 如收到 Task，则处理并发送 TaskAck 应答。
 */
int sapient_parse_and_handle_message(const char *data, size_t len, sapient_tcp_client_t *client)
{
    using namespace sapient_msg::bsi_flex_335_v2_0;

    SapientMessage msg;
    if (!msg.ParseFromArray(data, (int)len)) {
        LOGE("Failed to parse SapientMessage from received bytes\n");
        return -1;
    }

    // Check message type
    SapientMessage::ContentCase content_type = msg.content_case();
    switch (content_type) {
        case SapientMessage::kTask: {
            LOGI("Received Sapient Task message\n");
            // Extract Task and delegate to handler
            const Task &task = msg.task();
            std::string task_bin;
            if (!task.SerializeToString(&task_bin)) {
                LOGE("Failed to serialize Task for handler\n");
                return -1;
            }
            std::string ack_bin, ack_json;
            int action = TASK_ACTION_NONE;
            int ret = sapient_handle_task(task_bin.data(), task_bin.size(), ack_bin, ack_json, action);
            if (ret != 0) {
                LOGE("sapient_handle_task failed\n");
                return -1;
            }
            // Send TaskAck back
            LOGI("Sending TaskAck:\n%s\n", ack_json.c_str());
            if (client && client->impl) {
                client->impl->send_pb(ack_bin.data(), ack_bin.size());
                
                // 根据 action 类型执行相应动作
                if (action == TASK_ACTION_SEND_REGISTRATION) {
                    LOGI("Task requested Registration, sending Registration report\n");
                    sapient_tcp_client_send_register(client);
                    // 一次性任务执行完成，清除任务ID
                    sapient_clear_current_task_id();
                } else if (action == TASK_ACTION_SEND_STATUS) {
                    LOGI("Task requested Status, sending Status report\n");
                    sapient_tcp_client_send_status_report(client);
                    // 一次性任务执行完成，清除任务ID
                    sapient_clear_current_task_id();
                }
            }
            break;
        }
        case SapientMessage::kStatusReport:
            LOGI("Received Sapient StatusReport (informational)\n");
            break;
        case SapientMessage::kDetectionReport:
            LOGI("Received Sapient DetectionReport (informational)\n");
            break;
        case SapientMessage::kRegistrationAck:
            LOGI("Received Sapient RegistrationAck\n");
            // 标记收到 RegistrationAck，停止 30 秒超时计时器
            if (client && client->impl) {
                client->impl->mark_registration_ack_received();
                
                // 根据 SAPIENT 规范：收到 RegistrationAck 后，必须发送初始状态报告
                // "As part of system initialization, an initial status report message shall be 
                //  sent after the registration acknowledgement message has been received. 
                //  This shall indicate the initial state."
                LOGI("Sending initial status report after RegistrationAck (per SAPIENT spec)\n");
                int status_ret = client->impl->send_status_report();
                if (status_ret != 0) {
                    LOGE("Failed to send initial status report after RegistrationAck: %d\n", status_ret);
                } else {
                    LOGI("Initial status report sent successfully after RegistrationAck\n");
                }
            }
            break;
        case SapientMessage::kAlert:
            LOGI("Received Sapient Alert (not implemented)\n");
            break;
        default:
            LOGI("Received unhandled Sapient message type: %d\n", (int)content_type);
            break;
    }

    return 0;
}

} // extern "C"
