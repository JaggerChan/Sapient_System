#ifndef SAPIENT_NODEID_H
#define SAPIENT_NODEID_H

#include <string>

/* ===== 短期方案（当前）：强制 UUID v4 ===== */

/**
 * @brief 生成或获取设备的 NodeID（短期方案：强制 UUID v4）
 * 
 * @details 策略：文件 > UUID v4（持久化）
 * 1. 优先从持久化文件读取 NodeID（/home/chenyl/node_id.txt）
 * 2. 如果文件不存在：
 *    - **始终生成 UUID v4**（不管是否有序列号）
 *    - 持久化到文件，确保重启后保持不变
 * 
 * @note 短期方案目的：完全兼容客户系统（仅支持 UUID v4）
 * @note 一旦持久化，NodeID 永久固定
 * @note 如需重新生成，删除 /home/chenyl/node_id.txt 后重启
 * 
 * @return std::string UUID v4 格式的字符串（xxxxxxxx-xxxx-4xxx-xxxx-xxxxxxxxxxxx）
 */
std::string generateNodeID();

/* ===== 长期方案（未来）：可配置 UUID 版本 ===== */

/**
 * @brief 根据配置生成 NodeID（长期方案：支持 v4/v5 选择）
 * 
 * @details 策略：文件 > 配置选择（v4 或 v5）
 * 1. 优先从持久化文件读取
 * 2. 如果文件不存在：
 *    - use_uuid_v5 = true 且有 SN → 生成 UUID v5（基于 "SDH100" + SN）
 *    - use_uuid_v5 = false 或无 SN → 生成 UUID v4（随机）
 * 
 * @param use_uuid_v5 true: 使用 UUID v5, false: 使用 UUID v4
 * 
 * @note 长期方案：通过配置文件或 Web UI 让用户选择 UUID 版本
 * @note 未来集成到配置系统时使用此接口
 * 
 * @return std::string UUID 字符串
 */
std::string generateNodeIDWithConfig(bool use_uuid_v5);

#endif // SAPIENT_NODEID_H

