/*****************************************************************************
 * Copyright (c) 2023-2025
 * Skyfend Technology Co., Ltd
 *
 * @file    sapient_config_adapter.c
 * @brief   SAPIENT 配置适配器实现
 *****************************************************************************/
#include "sapient_config_adapter.h"
#include "../../common/cJSON/cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define LOG_TAG "sapient_config"
#include "../../common/zlog/skyfend_log.h"

/* 静态配置缓存 */
static sapient_config_t g_sapient_config = {0};
static char g_sapient_ip[64] = {0};
static int g_config_loaded = 0;

/* 从 CConfigManager 读取配置 */
static void load_sapient_config(void)
{
    if (g_config_loaded) {
        return;
    }

    /* 尝试从配置文件读取 */
    const char *config_path = "/home/root/sapient_config.json";
    
    FILE *fp = fopen(config_path, "r");
    if (!fp) {
        radar_log_warn("Cannot open config file: %s, SAPIENT disabled", config_path);
        g_config_loaded = 1;
        return;
    }

    fseek(fp, 0, SEEK_END);
    long len = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    
    char *json_str = (char *)malloc(len + 1);
    if (!json_str) {
        fclose(fp);
        return;
    }
    
    fread(json_str, 1, len, fp);
    json_str[len] = '\0';
    fclose(fp);

    cJSON *json = cJSON_Parse(json_str);
    if (!json) {
        radar_log_warn("Failed to parse config file");
        free(json_str);
        g_config_loaded = 1;
        return;
    }

    cJSON *sapient_obj = cJSON_GetObjectItem(json, "sapient");
    if (sapient_obj) {
        cJSON *ip_item = cJSON_GetObjectItem(sapient_obj, "ip");
        cJSON *port_item = cJSON_GetObjectItem(sapient_obj, "port");
        cJSON *enabled_item = cJSON_GetObjectItem(sapient_obj, "enabled");

        if (enabled_item && cJSON_IsFalse(enabled_item)) {
            radar_log_info("SAPIENT is disabled in config");
            cJSON_Delete(json);
            free(json_str);
            g_config_loaded = 1;
            return;
        }

        if (ip_item && cJSON_IsString(ip_item)) {
            strncpy(g_sapient_ip, ip_item->valuestring, sizeof(g_sapient_ip) - 1);
            g_sapient_config.ip = g_sapient_ip;
        }

        if (port_item && cJSON_IsNumber(port_item)) {
            g_sapient_config.port = port_item->valueint;
        }
    }

    cJSON_Delete(json);
    free(json_str);
    g_config_loaded = 1;

    if (g_sapient_config.ip && g_sapient_config.port > 0) {
        radar_log_info("SAPIENT config loaded: %s:%d", g_sapient_config.ip, g_sapient_config.port);
    } else {
        radar_log_info("SAPIENT config not found or incomplete");
    }
}

const sapient_config_t *sapient_config_get(void)
{
    load_sapient_config();
    
    if (g_sapient_config.ip && g_sapient_config.port > 0) {
        return &g_sapient_config;
    }
    
    return NULL;
}

