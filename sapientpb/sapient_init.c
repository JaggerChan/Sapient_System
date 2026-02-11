/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * All rights reserved. Any unauthorized disclosure or publication of the
 * confidential and proprietary information to any other party will constitute
 * an infringement of copyright laws.
 *
 * @file    sapient_init.c
 * @brief   SAPIENT 模块初始化实现（适配 acur101）
 *****************************************************************************/
#include "sapient_init.h"
#include "sapient_tcp.h"
#include "sapient_config_adapter.h"
#include "../../common/zlog/skyfend_log.h"
#include <stddef.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define LOG_TAG "sapient_init"

/* 全局 Sapient TCP 客户端句柄 */
static sapient_tcp_client_t *g_sapient_client = NULL;

/* ========== 后台重连机制 ========== */
static pthread_t g_reconnect_thread;
static int g_reconnect_thread_running = 0;
static pthread_mutex_t g_client_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ========== 状态报告定时发送机制 ========== */
static pthread_t g_status_report_thread;
static int g_status_report_thread_running = 0;
static const int STATUS_REPORT_INTERVAL = 5;  /* 每10秒发送一次状态报告 */
static const int STATUS_REPORT_DISCONNECT_THRESHOLD = 120;  /* 断网后2分钟内重连，不发送状态报告 */

/* 前置声明 */
static void start_status_report_thread(void);
static void stop_status_report_thread(void);

/* 验证 IP 地址格式是否合法 */
static int validate_ip(const char *ip)
{
	if (!ip || strlen(ip) == 0) return 0;
	struct in_addr addr;
	return inet_pton(AF_INET, ip, &addr) == 1;
}

/* 验证端口范围是否有效 */
static int validate_port(int port)
{
	return (port > 0 && port <= 65535);
}

/* 获取全局 Sapient TCP 客户端 */
sapient_tcp_client_t *get_sapient_client(void)
{
	return g_sapient_client;
}

/* Sapient 客户端接收回调：解析 SapientMessage，处理 Task 并回复 TaskAck */
#ifdef __cplusplus
extern "C" {
#endif
extern int sapient_parse_and_handle_message(const char *data, size_t len, sapient_tcp_client_t *client);
#ifdef __cplusplus
}
#endif

static void sapient_on_message(const char *data, size_t len, void *user)
{
	(void)user;
	radar_log_info("sapient client received %zu bytes", len);
	
	/* 解析并分发消息。如果是 Task，自动回复 TaskAck */
	pthread_mutex_lock(&g_client_mutex);
	if (g_sapient_client) {
		sapient_parse_and_handle_message(data, len, g_sapient_client);
	}
	pthread_mutex_unlock(&g_client_mutex);
}

/* ========== 后台重连线程 ========== */
static void *sapient_reconnect_thread(void *arg)
{
	(void)arg;
	int attempt = 0;
	const int retry_interval = 10;  /* 每10秒重试一次 */
	
	radar_log_info("sapient reconnect thread started");
	
	while (g_reconnect_thread_running && g_sapient_client) {
		attempt++;
		radar_log_info("sapient reconnect attempt %d", attempt);
		
		pthread_mutex_lock(&g_client_mutex);
		int cres = sapient_tcp_client_connect(g_sapient_client, 5);
		pthread_mutex_unlock(&g_client_mutex);
		
		if (cres == 0) {
			radar_log_info("sapient reconnect successful after %d attempts", attempt);
			
			/* 发送注册报文 */
			pthread_mutex_lock(&g_client_mutex);
			int sret = sapient_tcp_client_send_register(g_sapient_client);
			pthread_mutex_unlock(&g_client_mutex);
			
			if (sret != 0) {
				radar_log_error("sapient_tcp_client_send_register failed after reconnect: %d", sret);
			} else {
				radar_log_info("sapient register sent after reconnect");
			}
			
			/* 设置回调并启动接收线程 */
			pthread_mutex_lock(&g_client_mutex);
			sapient_tcp_client_set_on_message(g_sapient_client, sapient_on_message, NULL);
			int tret = sapient_tcp_client_start_receive_thread(g_sapient_client);
			pthread_mutex_unlock(&g_client_mutex);
			
			if (tret != 0) {
				radar_log_error("sapient_tcp_client_start_receive_thread failed after reconnect: %d", tret);
				/* 继续重试 */
				sleep(retry_interval);
				continue;
			} else {
				radar_log_info("sapient receive thread started after reconnect");
			}
			
			/* 启动状态报告线程（如果还没启动）
			 * 状态报告线程会根据断网时间自动判断是否需要发送
			 */
			start_status_report_thread();
			
			/* 连接成功，退出重连线程 */
			break;
		}
		
		radar_log_debug("sapient reconnect attempt %d failed, will retry in %d seconds", 
			 attempt, retry_interval);
		sleep(retry_interval);
	}
	
	g_reconnect_thread_running = 0;
	radar_log_info("sapient reconnect thread exited");
	return NULL;
}

/* ========== 状态报告定时发送线程 ========== */
static void *sapient_status_report_thread(void *arg)
{
	(void)arg;
	radar_log_info("sapient status report thread started");
	
	/* 等待一小段时间，确保连接和注册完成 */
	sleep(2);
	
	while (g_status_report_thread_running && g_sapient_client) {
		pthread_mutex_lock(&g_client_mutex);
		if (g_sapient_client) {
			int disconnect_elapsed = sapient_tcp_client_get_disconnect_elapsed_seconds(g_sapient_client);

			/* 按“断网 2 分钟规则”执行：
			 * - 如果 disconnect_elapsed 在 [0,120) ：不发送（哪怕已经重连成功，也继续抑制，直到满 120s）
			 * - 如果 disconnect_elapsed >= 120 ：允许发送一次，并清除断网计时，后续恢复正常周期发送
			 * - 如果 disconnect_elapsed < 0 ：无断网计时（首次连接或已清除），正常发送
			 */
			if (disconnect_elapsed >= 0 && disconnect_elapsed < STATUS_REPORT_DISCONNECT_THRESHOLD) {
				radar_log_debug("disconnect elapsed %d < %d, skip status report",
					disconnect_elapsed, STATUS_REPORT_DISCONNECT_THRESHOLD);
			} else {
				int ret = sapient_tcp_client_send_status_report(g_sapient_client);
				if (ret != 0) {
					radar_log_warn("sapient_tcp_client_send_status_report failed: %d", ret);
				} else {
					radar_log_debug("sapient status report sent (periodic)");
				}
				if (disconnect_elapsed >= STATUS_REPORT_DISCONNECT_THRESHOLD) {
					sapient_tcp_client_clear_disconnect_time(g_sapient_client);
					radar_log_info("disconnect elapsed %d >= %d, clear disconnect timer and resume normal status reporting",
						disconnect_elapsed, STATUS_REPORT_DISCONNECT_THRESHOLD);
				}
			}
		}
		pthread_mutex_unlock(&g_client_mutex);
		
		/* 等待指定间隔后再次发送 */
		for (int i = 0; i < STATUS_REPORT_INTERVAL && g_status_report_thread_running; i++) {
			sleep(1);
		}
	}
	
	radar_log_info("sapient status report thread exited");
	return NULL;
}

/* 启动状态报告线程 */
static void start_status_report_thread(void)
{
	if (g_status_report_thread_running) {
		radar_log_warn("status report thread already running");
		return;
	}
	
	g_status_report_thread_running = 1;
	if (pthread_create(&g_status_report_thread, NULL, sapient_status_report_thread, NULL) != 0) {
		radar_log_error("failed to create sapient status report thread");
		g_status_report_thread_running = 0;
	} else {
		pthread_detach(g_status_report_thread);
		radar_log_info("sapient status report thread created");
	}
}

/* 停止状态报告线程 */
static void stop_status_report_thread(void)
{
	if (g_status_report_thread_running) {
		int was_running = g_status_report_thread_running;
		g_status_report_thread_running = 0;
		radar_log_info("stopping sapient status report thread...");
		/* 等待线程退出（最多等待2秒） */
		for (int i = 0; i < 20 && was_running; i++) {
			usleep(100000);  /* 100ms */
		}
		radar_log_info("sapient status report thread stopped");
	}
}

/* Sapient 模块初始化
 * 读取配置、创建客户端、连接、发送注册报文、启动接收线程
 * 返回 0 表示成功，负值表示失败或配置未启用
 */
int sapient_init(void)
{
	const char *sapient_ip = NULL;
	int sapient_port = 0;

	/* 从配置适配器读取 Sapient 配置 */
	const sapient_config_t *cfg = sapient_config_get();
	if (cfg && cfg->ip && cfg->port > 0) {
		sapient_ip = cfg->ip;
		sapient_port = cfg->port;
		radar_log_info("Using Sapient config: %s:%d", sapient_ip, sapient_port);
	}

	/* 检查是否有有效配置 */
	if (!sapient_ip || sapient_port <= 0) {
		radar_log_info("SAPIENT ip/port not configured, skipping initialization");
		return SAPIENT_ERR_NOT_CONFIGURED;
	}

	/* 验证配置参数 */
	if (!validate_ip(sapient_ip)) {
		radar_log_error("invalid sapient ip address: %s", sapient_ip);
		return SAPIENT_ERR_NOT_CONFIGURED;
	}
	if (!validate_port(sapient_port)) {
		radar_log_error("invalid sapient port: %d (valid range: 1-65535)", sapient_port);
		return SAPIENT_ERR_NOT_CONFIGURED;
	}

	radar_log_info("creating sapient tcp client for %s:%d", sapient_ip, sapient_port);
	g_sapient_client = sapient_tcp_client_create(sapient_ip, sapient_port);
	if (!g_sapient_client) {
		radar_log_error("failed to create sapient tcp client");
		return SAPIENT_ERR_CREATE_FAILED;
	}

	/* ============ 初始连接尝试：重试3次，每次间隔5秒 ============ */
	int initial_success = 0;
	for (int retry = 0; retry < 3; retry++) {
		radar_log_info("sapient connect attempt %d/3", retry+1);
		
		int cres = sapient_tcp_client_connect(g_sapient_client, 5);
		if (cres == 0) {
			radar_log_info("sapient client connected on attempt %d", retry+1);
			
			/* 发送注册报文 */
			int sret = sapient_tcp_client_send_register(g_sapient_client);
			if (sret != 0) {
				radar_log_error("sapient_tcp_client_send_register failed: %d", sret);
			} else {
				radar_log_info("sapient register sent successfully");
			}
			
			/* 注册回调并启动后台接收线程 */
			sapient_tcp_client_set_on_message(g_sapient_client, sapient_on_message, NULL);
			int tret = sapient_tcp_client_start_receive_thread(g_sapient_client);
			if (tret != 0) {
				radar_log_error("sapient_tcp_client_start_receive_thread failed: %d", tret);
			} else {
				radar_log_info("sapient receive thread started");
				initial_success = 1;
				
				/* 注意：根据 SAPIENT 规范，初始状态报告应在收到 RegistrationAck 后发送
				 * 因此这里不再发送初始状态报告，而是在 sapient_tcp.cpp 的 
				 * sapient_parse_and_handle_message() 中，收到 RegistrationAck 时发送
				 */
				
				/* 启动状态报告定时发送线程 */
				start_status_report_thread();
			}
			break;
		}
		
		radar_log_warn("sapient connect attempt %d failed: %d", retry+1, cres);
		if (retry < 2) {
			radar_log_info("retrying in 5 seconds...");
			sleep(5);
		}
	}
	
	/* ============ 如果初始连接失败，启动后台重连线程 ============ */
	if (!initial_success) {
		radar_log_warn("sapient initial connect failed after 3 attempts (15s total)");
		radar_log_info("starting background reconnect thread...");
		
		g_reconnect_thread_running = 1;
		if (pthread_create(&g_reconnect_thread, NULL, sapient_reconnect_thread, NULL) != 0) {
			radar_log_error("failed to create sapient reconnect thread");
			g_reconnect_thread_running = 0;
		} else {
			pthread_detach(g_reconnect_thread);
			radar_log_info("sapient reconnect thread created");
		}
	}
	
	radar_log_info("sapient initialization completed (auto-reconnect enabled if needed)");
	return SAPIENT_OK;
}

/* Sapient 模块清理（如需要在退出时调用） */
void sapient_cleanup(void)
{
	/* 停止状态报告线程 */
	stop_status_report_thread();
	
	/* 停止后台重连线程 */
	if (g_reconnect_thread_running) {
		g_reconnect_thread_running = 0;
		radar_log_info("stopping sapient reconnect thread...");
		/* 等待线程退出（最多等待1秒） */
		for (int i = 0; i < 10 && g_reconnect_thread_running; i++) {
			usleep(100000);  /* 100ms */
		}
	}
	
	pthread_mutex_lock(&g_client_mutex);
	if (g_sapient_client) {
		sapient_tcp_client_stop_receive_thread(g_sapient_client);
		sapient_tcp_client_destroy(g_sapient_client);
		g_sapient_client = NULL;
		radar_log_info("sapient client cleaned up");
	}
	pthread_mutex_unlock(&g_client_mutex);
}

