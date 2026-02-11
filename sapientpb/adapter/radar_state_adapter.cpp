/**
 * @file radar_state_adapter.cpp
 * @brief RadarState 数据获取实现（正确方案：从 Alink 数据通道截取）
 * @note 符合架构设计原则：
 *   - Data Adapter 获取的是**已经处理过、融合好的数据**
 *   - 在数据通过 Alink Protocol 发送之前被截取
 *   - 不重复聚合数据，性能最优
 *   - 与 Alink 协议解耦（截取发生在发送之前，不依赖网络传输）
 * 
 * 数据来源：
 *   - Sub_TrackAttitude() 填充的 RadarState（已融合）
 *   - 在 Pub_Track_Attitude() 之前截取（复用已整理好的数据）
 */

#include "radar_state_adapter.h"
#include "../../../common/zlog/skyfend_log.h"
#include "../../../cfg/ConfigManager.h"
#include "../../../radar_front/pl/pl_reg.h"
#include "../../../app/DataPath/data_path.h"
#include <string.h>
#include <pthread.h>
#include <cmath>

// 辅助函数：获取两个浮点数的最大值
static inline float max_float(float a, float b) {
    return (a > b) ? a : b;
}

#define LOG_TAG "radar_state_adapter"

// 全局变量：保存最新的 RadarState（从 Alink 数据通道截取）
static RadarState g_latest_radar_state;
static pthread_mutex_t g_radar_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_radar_state_valid = false;

/**
 * @brief 从 Alink 数据通道截取 RadarState（在发送前调用）
 * @param state 已填充的 RadarState 指针
 * @note 此函数应该在 Sub_TrackAttitude() 之后、Pub_Track_Attitude() 之前调用
 *       这样可以在数据发送到 Alink 之前截取，复用已整理好的数据，性能最优
 */
extern "C" void capture_radar_state_for_sapient(const RadarState *state)
{
    if (!state) {
        radar_log_error("state pointer is NULL");
        return;
    }
    
    pthread_mutex_lock(&g_radar_state_mutex);
    memcpy(&g_latest_radar_state, state, sizeof(RadarState));
    g_radar_state_valid = true;
    pthread_mutex_unlock(&g_radar_state_mutex);
    
    radar_log_debug("Captured RadarState from Alink data path (msgid=0x20)");
}

/**
 * @brief 获取截取的 RadarState 数据（供 SAPIENT 使用）
 * @param state 输出参数，填充 RadarState 结构体
 * @return 0 成功，-1 失败
 * @note 数据来源：从 Alink Protocol 数据通道截取的 RadarState (msgid=0x20)
 *       在 Pub_Track_Attitude() 之前截取，复用已整理好的数据，性能最优
 */
extern "C" int get_radar_state(RadarState *state)
{
    if (!state) {
        radar_log_error("state pointer is NULL");
        return -1;
    }
    
    pthread_mutex_lock(&g_radar_state_mutex);
    
    if (!g_radar_state_valid) {
        pthread_mutex_unlock(&g_radar_state_mutex);
        radar_log_warn("No valid RadarState data captured yet");
        return -1;
    }
    
    memcpy(state, &g_latest_radar_state, sizeof(RadarState));
    pthread_mutex_unlock(&g_radar_state_mutex);
    
    radar_log_debug("RadarState retrieved from captured data (Alink path)");
    
    return 0;
}

/**
 * @brief 温度码转浮点数（与 device_info.cpp 中的实现一致）
 */
static inline float TempCode2Float(long code)
{
    return 507.5921310f * code / 65536.0f - 279.42657680f;
}

/**
 * @brief 从温度监控模块获取雷达温度
 * @note 温度数据通过 Alink Protocol 的 0x43 (TemperatureAll) 上报，
 *       但 RadarState (0x20) 不包含温度字段，所以需要单独获取
 * @return 雷达温度（摄氏度），如果获取失败返回 0.0f
 * 
 * 策略：获取所有温度（RF 天线温度 + FPGA 温度），返回最高温度
 * 原因：最高温度最能反映系统热状态，用于判断是否过热
 */
extern "C" float get_radar_temperature(void)
{
    float max_temp = -100.0f;  // 初始化为一个很低的温度
    bool has_valid_temp = false;
    
    // 1. 获取 RF 天线温度
    float temperature[16];
    float temperature_u8;
    GetRfTempAll(&temperature_u8, temperature);
    
    // 验证并找到 RF 温度中的最高值
    if (std::isfinite(temperature_u8) && temperature_u8 > -50.0f && temperature_u8 < 150.0f) {
        max_temp = max_float(max_temp, temperature_u8);
        has_valid_temp = true;
    }
    
    for (int i = 0; i < 16; i++) {
        if (std::isfinite(temperature[i]) && temperature[i] > -50.0f && temperature[i] < 150.0f) {
            max_temp = max_float(max_temp, temperature[i]);
            has_valid_temp = true;
        }
    }
    
    // 2. 获取 FPGA 温度
    uint32_t plTemp = PlGetTemperature();
    float fpga_temp = TempCode2Float(plTemp);
    if (std::isfinite(fpga_temp) && fpga_temp > -50.0f && fpga_temp < 150.0f) {
        max_temp = max_float(max_temp, fpga_temp);
        has_valid_temp = true;
    }
    
    // 3. 验证温度合理性
    if (!has_valid_temp || max_temp < -50.0f || max_temp > 150.0f || !std::isfinite(max_temp)) {
        radar_log_warn("Invalid temperature value: %.2f (no valid sensor data), returning 0", max_temp);
        return 0.0f;
    }
    
    radar_log_debug("Radar temperature: %.2f°C (max of RF and FPGA)", max_temp);
    return max_temp;
}

/**
 * @brief 获取杂波抑制状态
 * @note 从系统配置获取杂波相关状态
 */
extern "C" int get_clutter_status(clutter_status_t *status)
{
    if (!status) {
        radar_log_error("status pointer is NULL");
        return -1;
    }
    
    // 初始化为默认值
    memset(status, 0, sizeof(clutter_status_t));
    
    // 从系统配置获取杂波相关状态
    tCfgSystem syscfg = CConfigManager::GetInstance()->GetSystemCfg();
    status->filter_level = syscfg.filterLevel;
    status->weather_clutter_filter = syscfg.meteCluterFilter;
    
    radar_log_debug("Filter level: %d, Weather clutter filter: %d", 
                    status->filter_level, 
                    status->weather_clutter_filter);
    
    return 0;
}

/**
 * @brief 获取雷达侦测开关状态
 * @note 从系统配置获取 trackEnabled 字段
 */
extern "C" int get_track_enabled_status(void)
{
    tCfgSystem syscfg = CConfigManager::GetInstance()->GetSystemCfg();
    int track_enabled = syscfg.trackEnabled ? 1 : 0;
    
    radar_log_debug("Track enabled: %d", track_enabled);
    
    return track_enabled;
}

/**
 * @brief 获取 OTM 模式状态
 * @note 从系统配置获取 otmMode 字段
 */
extern "C" int get_otm_mode_status(void)
{
    tCfgSystem syscfg = CConfigManager::GetInstance()->GetSystemCfg();
    int otm_mode = syscfg.otmMode ? 1 : 0;
    
    radar_log_debug("OTM mode: %d", otm_mode);
    
    return otm_mode;
}
