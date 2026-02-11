/**
 * @file radar_state_adapter.h
 * @brief RadarState 数据适配器接口
 * @note 从 Alink Protocol 数据通道截取 RadarState（已融合）
 *       在 Pub_Track_Attitude() 之前截取，复用已整理好的数据，性能最优
 */

#ifndef __RADAR_STATE_ADAPTER_H__
#define __RADAR_STATE_ADAPTER_H__

#include "../../../common/nanopb/radar.pb.h" // For RadarState

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 从 Alink 数据通道截取 RadarState（在发送前调用）
 * @param state 已填充的 RadarState 指针
 * @note 此函数应该在 Sub_TrackAttitude() 之后、Pub_Track_Attitude() 之前调用
 *       这样可以在数据发送到 Alink 之前截取，复用已整理好的数据，性能最优
 */
void capture_radar_state_for_sapient(const RadarState *state);

/**
 * @brief 获取最新的雷达状态数据（供 SAPIENT 使用）
 * @param state 输出参数，填充 RadarState 结构体
 * @return 0 成功，-1 失败
 * @note 数据来源：从 Alink Protocol 数据通道截取的 RadarState (msgid=0x20)
 *       在 Pub_Track_Attitude() 之前截取，复用已整理好的数据，性能最优
 */
int get_radar_state(RadarState *state);

/**
 * @brief 获取雷达板载温度
 * @return 温度值（°C），如果失败返回 0.0f
 * @note 温度不在 RadarState (0x20) 中，需要单独获取
 */
float get_radar_temperature(void);

/**
 * @brief 杂波抑制状态结构体
 */
typedef struct {
    int filter_level;             // 滤波等级 (杂波相关)
    int weather_clutter_filter;   // 气象杂波抑制 (0=关闭, 1=开启)
} clutter_status_t;

/**
 * @brief 获取杂波抑制状态
 * @param status 输出参数，填充杂波状态结构体
 * @return 0 成功，-1 失败
 * @note 从系统配置和协议层参数获取杂波相关状态
 */
int get_clutter_status(clutter_status_t *status);

/**
 * @brief 获取雷达侦测开关状态
 * @return 1 侦测已启用，0 侦测未启用
 * @note 对应 ConfigManager 中的 trackEnabled 字段
 */
int get_track_enabled_status(void);

/**
 * @brief 获取 OTM 模式状态
 * @return 1 OTM 模式已启用，0 OTM 模式未启用
 * @note 对应 ConfigManager 中的 otmMode 字段
 */
int get_otm_mode_status(void);

#ifdef __cplusplus
}
#endif

#endif // __RADAR_STATE_ADAPTER_H__
