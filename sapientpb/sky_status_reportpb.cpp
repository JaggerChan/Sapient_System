#include <iostream>
#include <string>
#include <chrono>
#include <cstdint>
#include <limits>
#include <cmath>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/timestamp.pb.h>
#include "../sapient/status_report.pb.h"
#include "../sapient/sapient_message.pb.h"
#include "sapient_nodeid.h"

extern std::string getCurrentTimeISO8601();

extern "C" {
    #include <stdio.h>
    #include <stdlib.h>
    #include <time.h>
    #include <string.h>
    #include "adapter/radar_state_adapter.h"
}

// 声明在 sky_detection_reportpb.cpp 中实现的 ULID 生成函数
extern "C" void generate_ulid(char *ulid);

// 声明在 sky_task_handler.cpp 中实现的任务查询函数（C++ 函数）
#include "sky_task_handler.h"

// 简化的状态快照结构体：用于判断关键状态是否变化
struct StatusSnapshot {
    // 基础状态
    uint32_t sysStatus;        // 系统状态
    uint32_t faultCount;       // 故障数量
    uint8_t maxFaultLevel;     // 最高故障级别
    
    // 位置和姿态
    double longitude;          // 经度
    double latitude;           // 纬度
    double altitude;           // 海拔
    double heading;            // 航向
    double pitching;           // 俯仰
    double rolling;            // 横滚
    
    // 功能状态
    bool trackEnabled;         // 检测开关状态
    bool otmMode;              // OTM 模式状态
    
    // 杂波抑制状态
    uint32_t filterLevel;      // 杂波滤波等级
    bool weatherClutterFilter; // 气象杂波抑制
    
    // 温度（影响系统安全）
    float temperature;         // 温度
    
    bool operator==(const StatusSnapshot& other) const {
        const double POSITION_EPSILON = 0.00001;  // 约1米精度
        const double ANGLE_EPSILON = 0.1;          // 0.1度精度
        const float TEMP_EPSILON = 5.0f;           // 5°C 容差（避免频繁变化）
        
        return sysStatus == other.sysStatus &&
               faultCount == other.faultCount &&
               maxFaultLevel == other.maxFaultLevel &&
               std::fabs(longitude - other.longitude) < POSITION_EPSILON &&
               std::fabs(latitude - other.latitude) < POSITION_EPSILON &&
               std::fabs(altitude - other.altitude) < POSITION_EPSILON &&
               std::fabs(heading - other.heading) < ANGLE_EPSILON &&
               std::fabs(pitching - other.pitching) < ANGLE_EPSILON &&
               std::fabs(rolling - other.rolling) < ANGLE_EPSILON &&
               trackEnabled == other.trackEnabled &&
               otmMode == other.otmMode &&
               filterLevel == other.filterLevel &&
               weatherClutterFilter == other.weatherClutterFilter &&
               std::fabs(temperature - other.temperature) < TEMP_EPSILON;
    }
};

// 全局保存上一次的状态（初始化为无效值，确保首次报告为 INFO_NEW）
static StatusSnapshot last_snapshot = {
    0xFFFFFFFF, 0xFFFFFFFF, 0xFF,  // sysStatus, faultCount, maxFaultLevel
    999.0, 999.0, 999.0,            // longitude, latitude, altitude
    999.0, 999.0, 999.0,            // heading, pitching, rolling
    false, false,                    // trackEnabled, otmMode
    0xFFFFFFFF, false,              // filterLevel, weatherClutterFilter
    -999.0f                          // temperature
};

// 解析雷达状态位（Bit 定义参见 RadarState.status 注释）
static void parse_radar_status(uint32_t status, 
                               uint8_t *motion_state,    // B2B1B0: 静止/运动/转动
                               uint8_t *platform_type,   // B5B4B3: 固定式/转台式/车载式/机载式
                               uint8_t *detection_mode,  // B8B7B6: 无人机/行人/车船模式
                               uint8_t *power_type,      // B10B9: 固定电源/电池
                               uint8_t *network_speed,   // B12B11: 网络速度
                               uint8_t *power_mode,      // B14B13: 默认/低功耗
                               uint8_t *attitude_source) // B16B15: 姿态数据来源
{
    *motion_state = (status >> 0) & 0x07;   // Bit 0-2
    *platform_type = (status >> 3) & 0x07;  // Bit 3-5
    *detection_mode = (status >> 6) & 0x07; // Bit 6-8
    *power_type = (status >> 9) & 0x03;     // Bit 9-10
    *network_speed = (status >> 11) & 0x03; // Bit 11-12
    *power_mode = (status >> 13) & 0x03;    // Bit 13-14
    *attitude_source = (status >> 15) & 0x03; // Bit 15-16
}

// C++ 构建函数：生成二进制 protobuf 和 JSON（用于日志/调试）
// 构造 StatusReport，封装进 SapientMessage wrapper，返回序列化的 wrapper
int sapient_build_status_report(std::string &out_serialized, std::string &out_json)
{
    // 使用生成的 protobuf 类型 StatusReport
    sapient_msg::bsi_flex_335_v2_0::StatusReport statusrepo;

    // 生成并设置 report_id
    char ulid[27] = {0};
    generate_ulid(ulid);
    statusrepo.set_report_id(ulid);

    // 设置 active_task_id（当前执行的任务 ID）
    std::string current_task_id = sapient_get_current_task_id();
    if (!current_task_id.empty()) {
        statusrepo.set_active_task_id(current_task_id);
    }

    // ======================== 获取雷达状态数据 ========================
    RadarState radar_state;
    memset(&radar_state, 0, sizeof(RadarState));
    int ret = get_radar_state(&radar_state);
    if (ret != 0) {
        std::cerr << "Warning: Failed to get radar state, using default values" << std::endl;
    }

    // 提取关键字段，构建当前状态快照
    uint8_t maxFaultLevel = 0;
    if (radar_state.faultCount > 0) {
        for (uint32_t i = 0; i < radar_state.faultCount && i < 64; i++) {
            if (radar_state.fault[i].faultLevel > maxFaultLevel) {
                maxFaultLevel = radar_state.fault[i].faultLevel;
            }
        }
    }
    
    // 获取功能状态
    int track_enabled = get_track_enabled_status();
    int otm_mode = get_otm_mode_status();
    
    // 获取杂波抑制状态
    clutter_status_t clutter_status;
    memset(&clutter_status, 0, sizeof(clutter_status_t));
    get_clutter_status(&clutter_status);
    
    // 获取温度
    float temperature = get_radar_temperature();

    StatusSnapshot current = {
        // 基础状态
        radar_state.has_sysStatus ? radar_state.sysStatus : 0,
        radar_state.faultCount,
        maxFaultLevel,
        // 位置和姿态
        radar_state.has_radarLLA ? radar_state.radarLLA.longitude : 0.0,
        radar_state.has_radarLLA ? radar_state.radarLLA.latitude : 0.0,
        radar_state.has_radarLLA ? radar_state.radarLLA.altitude : 0.0,
        (radar_state.has_attitude && radar_state.attitude.has_heading) ? radar_state.attitude.heading : 0.0,
        (radar_state.has_attitude && radar_state.attitude.has_pitching) ? radar_state.attitude.pitching : 0.0,
        (radar_state.has_attitude && radar_state.attitude.has_rolling) ? radar_state.attitude.rolling : 0.0,
        // 功能状态
        static_cast<bool>(track_enabled),
        static_cast<bool>(otm_mode),
        // 杂波抑制状态
        static_cast<uint32_t>(clutter_status.filter_level),
        static_cast<bool>(clutter_status.weather_clutter_filter),
        // 温度
        temperature
    };

    // 判断 info 字段：状态是否有变化
    if (current == last_snapshot) {
        statusrepo.set_info(sapient_msg::bsi_flex_335_v2_0::StatusReport_Info_INFO_UNCHANGED);
    } else {
        statusrepo.set_info(sapient_msg::bsi_flex_335_v2_0::StatusReport_Info_INFO_NEW);
        last_snapshot = current;  // 更新上次快照
    }

    // ======================== system 映射 ========================
    // 根据最高故障级别判断系统状态
    sapient_msg::bsi_flex_335_v2_0::StatusReport_System sys_enum;
    if (maxFaultLevel == 0x03) {
        // 0x03: 无法使用
        sys_enum = sapient_msg::bsi_flex_335_v2_0::StatusReport_System_SYSTEM_ERROR;
    } else if (maxFaultLevel == 0x02) {
        // 0x02: 功能受限
        sys_enum = sapient_msg::bsi_flex_335_v2_0::StatusReport_System_SYSTEM_WARNING;
    } else if (maxFaultLevel == 0x01) {
        // 0x01: 告警信息
        sys_enum = sapient_msg::bsi_flex_335_v2_0::StatusReport_System_SYSTEM_WARNING;
    } else if (radar_state.has_sysStatus && 
               (radar_state.sysStatus == 3 || radar_state.sysStatus == 4 || radar_state.sysStatus == 5)) {
        // 系统状态：待机/正常探测/搜索模式
        sys_enum = sapient_msg::bsi_flex_335_v2_0::StatusReport_System_SYSTEM_OK;
    } else {
        // 其他状态：初始化、自检等
        sys_enum = sapient_msg::bsi_flex_335_v2_0::StatusReport_System_SYSTEM_UNSPECIFIED;
    }
    statusrepo.set_system(sys_enum);

    // ======================== mode 映射 ========================
    // 根据系统状态 (sysStatus) 和雷达状态位 (status) 综合判断
    std::string mode_str = "unknown";
    if (radar_state.has_sysStatus) {
        switch (radar_state.sysStatus) {
            case 0: mode_str = "default"; break;
            case 1: mode_str = "initializing"; break;
            case 2: mode_str = "self_checking"; break;
            case 3: mode_str = "standby"; break;
            case 4: mode_str = "normal_detection"; break;  // TAS/TWS切换
            case 5: mode_str = "search_mode"; break;       // TWS
            case 6: mode_str = "fire_control"; break;      // 预留
            case 11: mode_str = "test_mode"; break;
            case 22: mode_str = "factory_mode"; break;
            case 33: mode_str = "mesh_network"; break;
            case 99: mode_str = "error"; break;
            default: mode_str = "unknown"; break;
        }
    }
    statusrepo.set_mode(mode_str);

    // ======================== node_location（设备节点位置）========================
    if (radar_state.has_radarLLA && 
        (radar_state.radarLLA.longitude != 0.0 || radar_state.radarLLA.latitude != 0.0)) {
        auto *node_loc = statusrepo.mutable_node_location();
        node_loc->set_x(radar_state.radarLLA.longitude);  // 经度（度）
        node_loc->set_y(radar_state.radarLLA.latitude);   // 纬度（度）
        node_loc->set_z(radar_state.radarLLA.altitude);   // 海拔（米）
        
        // 调试日志：打印位置数据（现在始终来自 GNSS 实时数据）
        // 注意：这里使用 std::cout 或 radar_log_info，根据项目日志系统选择
        // radar_log_info("[SAPIENT_MODE] StatusReport location (from GNSS): lon=%.6f, lat=%.6f, alt=%.2f", 
        //               radar_state.radarLLA.longitude, radar_state.radarLLA.latitude, radar_state.radarLLA.altitude);
        
        // 位置误差（假设 6 米精度）
        const double METERS_PER_DEGREE = 111000.0;
        const double ERROR_METERS = 6.0;
        double error_deg = std::round((ERROR_METERS / METERS_PER_DEGREE) * 100000.0) / 100000.0;
        
        node_loc->set_x_error(error_deg);   // 经度误差（度）
        node_loc->set_y_error(error_deg);   // 纬度误差（度）
        //node_loc->set_z_error(10.0);        // 海拔误差（米）
        node_loc->set_coordinate_system(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
        node_loc->set_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);
    }

    // ======================== node_velocity 和 node_acceleration ========================
    // 注意：StatusReport protobuf 定义中没有 node_velocity 和 node_acceleration 字段
    // 如果需要报告速度/加速度信息，可以添加到 status 条目中

    // ======================== power（电源信息）========================
    {
        auto *power = statusrepo.mutable_power();
        
        // 解析电源类型（从 status 位 B10B9）
        uint8_t motion, platform, detect_mode, power_type, net_speed, pwr_mode, att_src;
        if (radar_state.has_status) {
            parse_radar_status(radar_state.status, &motion, &platform, &detect_mode, 
                             &power_type, &net_speed, &pwr_mode, &att_src);
            
            if (power_type == 0x00) {
                // 固定电源
                power->set_source(sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerSource_POWERSOURCE_MAINS);
            } else if (power_type == 0x01) {
                // 电池（假设为内部电池）
                power->set_source(sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerSource_POWERSOURCE_INTERNAL_BATTERY);
            } else {
                power->set_source(sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerSource_POWERSOURCE_UNSPECIFIED);
            }
        } else {
            // 默认：市电
            power->set_source(sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerSource_POWERSOURCE_MAINS);
        }
        
        // 电源状态：根据电源类型和电量百分比判断
        sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerStatus pwr_status;
        if (radar_state.has_status && power_type == 0x01 && radar_state.has_electricity) {
            // 电池供电：根据电量判断状态
            if (radar_state.electricity > 20) {
                pwr_status = sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerStatus_POWERSTATUS_OK;
            } else {
                // 电量低于 20% 视为故障
                pwr_status = sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerStatus_POWERSTATUS_FAULT;
            }
            // 只在电池供电时设置电量百分比
            power->set_level(static_cast<int32_t>(radar_state.electricity));
        } else {
            // 市电供电：始终为 OK（不设置 level 字段）
            pwr_status = sapient_msg::bsi_flex_335_v2_0::StatusReport_PowerStatus_POWERSTATUS_OK;
        }
        power->set_status(pwr_status);
    }

    // ======================== field_of_view（视场角）========================
    // 根据扫描参数计算
    // field_of_view 是 LocationOrRangeBearing 类型，使用 RangeBearingCone
    if (radar_state.has_aziScanCenter && radar_state.has_aziScanScope &&
        radar_state.has_eleScanCenter && radar_state.has_eleScanScope &&
        radar_state.has_radarScanRadius) {
        
        auto *fov = statusrepo.mutable_field_of_view();
        auto *range_bearing = fov->mutable_range_bearing();
        

        // 计算相对于水平面的角度（波束角度 + 平台姿态角）
        // SAPIENT 要求：elevation 是相对于水平面的角度（Angle above/below horizontal）
        double platform_heading = 0.0;  // 雷达平台航向角
        double platform_pitching = 0.0; // 雷达平台俯仰角
        
        if (radar_state.has_attitude) {
            if (radar_state.attitude.has_heading) {
                platform_heading = radar_state.attitude.heading;
            }
            if (radar_state.attitude.has_pitching) {
                platform_pitching = radar_state.attitude.pitching;
            }
        }
        
        // 中心方位角：波束扫描中心 + 平台航向角（相对于正北）
        double azimuth_relative_to_north = radar_state.aziScanCenter + platform_heading;
        // 归一化到 [0, 360)
        if (azimuth_relative_to_north < 0.0) {
            azimuth_relative_to_north += 360.0;
        } else if (azimuth_relative_to_north >= 360.0) {
            azimuth_relative_to_north -= 360.0;
        }
        range_bearing->set_azimuth(azimuth_relative_to_north);
        
        // 中心俯仰角：波束扫描中心 + 平台俯仰角（相对于水平面）
        double elevation_relative_to_horizontal = radar_state.eleScanCenter + platform_pitching;
        range_bearing->set_elevation(elevation_relative_to_horizontal);
        
        // 距离范围
        range_bearing->set_range(static_cast<double>(radar_state.radarScanRadius));
        
        // 水平范围（方位角范围）
        range_bearing->set_horizontal_extent(static_cast<double>(radar_state.aziScanScope));
        // 垂直范围（俯仰角范围）
        range_bearing->set_vertical_extent(static_cast<double>(radar_state.eleScanScope));
        
        // 坐标系和基准
        range_bearing->set_coordinate_system(
            sapient_msg::bsi_flex_335_v2_0::RANGE_BEARING_COORDINATE_SYSTEM_DEGREES_M);
        range_bearing->set_datum(
            sapient_msg::bsi_flex_335_v2_0::RANGE_BEARING_DATUM_TRUE);
    }

    // ======================== status（状态条目列表）========================
    // helper: 添加 status 条目
    auto add_status = [&statusrepo](sapient_msg::bsi_flex_335_v2_0::StatusReport_StatusLevel level,
                                    sapient_msg::bsi_flex_335_v2_0::StatusReport_StatusType type,
                                    const std::string &value) {
        auto *s = statusrepo.add_status();
        s->set_status_level(level);
        s->set_status_type(type);
        s->set_status_value(value);
    };

    using namespace sapient_msg::bsi_flex_335_v2_0;
    
    // 1. 平台类型（从 status 位 B5B4B3 解析）
    if (radar_state.has_status) {
        uint8_t motion, platform, detect_mode, power_type, net_speed, pwr_mode, att_src;
        parse_radar_status(radar_state.status, &motion, &platform, &detect_mode, 
                         &power_type, &net_speed, &pwr_mode, &att_src);
        
        const char *platform_str = "Unknown";
        switch (platform) {
            case 0x00: platform_str = "Fixed"; break;           // 固定式
            case 0x01: platform_str = "Fixed_Turntable"; break; // 固定转台式
            case 0x02: platform_str = "Vehicle_Mounted"; break; // 车载式/舰载式
            case 0x03: platform_str = "Airborne"; break;        // 机载式
            default: platform_str = "Unknown"; break;
        }
        add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                   StatusReport_StatusType_STATUS_TYPE_PLATFORM,
                   platform_str);
        
        // 姿态数据来源（从 status 位 B16B15 解析）
        const char *attitude_source_str = "Unknown";
        switch (att_src) {
            case 0x00: attitude_source_str = "Radar_Attitude_System"; break;           // 使用雷达姿态系统
            case 0x01: attitude_source_str = "Radar_Attitude_System_Calibrated"; break; // 使用雷达姿态系统标定后静止参数
            case 0x02: attitude_source_str = "External_Attitude_Input"; break;          // 使用外部输入的姿态数据
            default: attitude_source_str = "Unknown"; break;
        }
        add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                   StatusReport_StatusType_STATUS_TYPE_PLATFORM,
                   attitude_source_str);
    }
    
    // 1.5. 检测状态（trackEnabled）
    // 注意：track_enabled 已在快照构建时获取，这里直接使用
    // if (!track_enabled) {
    //     // 如果雷达未启动侦测，报告 NOT_DETECTING
    //     add_status(StatusReport_StatusLevel_STATUS_LEVEL_WARNING_STATUS,
    //                StatusReport_StatusType_STATUS_TYPE_NOT_DETECTING,
    //                "Radar_Detection_Disabled");
    // }
    
    // 1.6. OTM 模式（运动灵敏度相关）
    // 注意：otm_mode 已在快照构建时获取，这里直接使用
    if (otm_mode) {
        add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                   StatusReport_StatusType_STATUS_TYPE_MOTION_SENSITIVITY,
                   "OTM_Mode_Enabled");
    } else {
        add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                   StatusReport_StatusType_STATUS_TYPE_MOTION_SENSITIVITY,
                   "OTM_Mode_Disabled");
    }

    // 2. 杂波抑制状态（Clutter）
    // 注意：clutter_status 已在快照构建时获取，这里直接使用
    if (get_clutter_status(&clutter_status) == 0) {
        char clutter_str[128];
        
        // 滤波等级（杂波相关）
        snprintf(clutter_str, sizeof(clutter_str), "Filter_Level=%d", clutter_status.filter_level);
        add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                  StatusReport_StatusType_STATUS_TYPE_CLUTTER,
                  clutter_str);
        
        // 气象杂波抑制状态
        if (clutter_status.weather_clutter_filter) {
            add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                      StatusReport_StatusType_STATUS_TYPE_CLUTTER,
                      "Weather_Clutter_Filter=Enabled");
        } else {
            add_status(StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS,
                      StatusReport_StatusType_STATUS_TYPE_CLUTTER,
                      "Weather_Clutter_Filter=Disabled");
        }
    }

    // 3. 温度
    // 注意：temperature 已在快照构建时获取，这里直接使用
    if (temperature > 0.0f) {
        char temp_str[64];
        snprintf(temp_str, sizeof(temp_str), "Temperature=%.1f°C", temperature);
        
        StatusReport_StatusLevel temp_level;
        if (temperature > 80.0f) {
            temp_level = StatusReport_StatusLevel_STATUS_LEVEL_ERROR_STATUS;
        } else if (temperature > 70.0f) {
            temp_level = StatusReport_StatusLevel_STATUS_LEVEL_WARNING_STATUS;
        } else {
            temp_level = StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS;
        }
        add_status(temp_level, StatusReport_StatusType_STATUS_TYPE_OTHER, temp_str);
    }

    // 9. 故障信息
    if (radar_state.faultCount > 0) {
        for (uint32_t i = 0; i < radar_state.faultCount && i < 64; i++) {
            StatusReport_StatusLevel fault_level;
            switch (radar_state.fault[i].faultLevel) {
                case 0x01: fault_level = StatusReport_StatusLevel_STATUS_LEVEL_WARNING_STATUS; break;
                case 0x02: fault_level = StatusReport_StatusLevel_STATUS_LEVEL_WARNING_STATUS; break;
                case 0x03: fault_level = StatusReport_StatusLevel_STATUS_LEVEL_ERROR_STATUS; break;
                default: fault_level = StatusReport_StatusLevel_STATUS_LEVEL_INFORMATION_STATUS; break;
            }
            
            char fault_str[128];
            snprintf(fault_str, sizeof(fault_str), "Fault_Code=0x%04X, Level=0x%02X", 
                     radar_state.fault[i].faultCode, radar_state.fault[i].faultLevel);
            add_status(fault_level, StatusReport_StatusType_STATUS_TYPE_INTERNAL_FAULT, fault_str);
        }
    }

    // ======================== 构造 SapientMessage wrapper ========================
    sapient_msg::bsi_flex_335_v2_0::SapientMessage wrapper;
    std::string node_id = generateNodeID();
    if (node_id.size() > 0) {
        wrapper.set_node_id(node_id);
    }
    
    // 顶层 timestamp 使用 google::protobuf::Timestamp 类型
    {
        auto now = std::chrono::system_clock::now();
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        auto nanos_total = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        int nanos = static_cast<int>(nanos_total - secs * 1000000000LL);
        google::protobuf::Timestamp* ts = wrapper.mutable_timestamp();
        ts->set_seconds(static_cast<long long>(secs));
        ts->set_nanos(nanos);
    }
    
    // 把 status_report 字段拷入 wrapper 的 status_report oneof
    wrapper.mutable_status_report()->CopyFrom(statusrepo);

    // 序列化 wrapper 到二进制
    if (!wrapper.SerializeToString(&out_serialized)) {
        std::cerr << "序列化 SapientMessage wrapper 失败" << std::endl;
        return -1;
    }

    // 序列化 wrapper 到 JSON（用于调试/日志）
    google::protobuf::util::JsonOptions options; 
    options.add_whitespace = true;
    auto status = google::protobuf::util::MessageToJsonString(wrapper, &out_json, options);
    if (!status.ok()) {
        std::cerr << "将 wrapper message 转换为 JSON 失败: " << status.ToString() << std::endl;
        return -1;
    }

    return 0;
}

extern "C" {
    // C-compatible wrapper（可选，用于兼容性或测试）
    int sapient_status_report(void) 
    {
        std::string bin, json;
        int rc = sapient_build_status_report(bin, json);
        if (rc != 0) return rc;
        std::cout << "Serialized JSON output: " << std::endl;
        std::cout << json << std::endl;
        return 0;
    }
}
