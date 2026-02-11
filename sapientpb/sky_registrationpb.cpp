#include "../sapient/sky_registration.pb.h"
#include "../sapient/sapient_message.pb.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <random>
#include <math.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/timestamp.pb.h>

extern "C" {
    #include "../../inc/GNSS_coordinate.h"
    #include "adapter/auto_hunt_param_adapter.h"
    #include "adapter/sn_adapter.h"
    #include "../../srv/version/version.h"
}

#include "sapient_nodeid.h"
#include "sapient_product.h"

// std::string g_nodeId; // Removed, use generateNodeID() instead
std::string g_sn;  // 设备序列号全局变量（定义，而非声明）

static void getSn(void)
{
    char buffer[SN_MAX_SIZE+1]={0};
    int ret = read_sn(buffer, SN_MAX_SIZE);
    if (ret < 0) {
        std::cout << "read_sn failed" << std::endl;
        return;
    }
    g_sn = std::string(buffer);
}

/**
 * @brief 生成 ISO 8601 格式的 UTC 时间戳字符串
 * 
 * @details 该函数获取当前系统时间，转换为 UTC 时区，并格式化为符合 ISO 8601 标准的时间戳字符串。
 *          时间戳包含毫秒精度，格式为：YYYY-MM-DDTHH:MM:SS.mmmZ
 * 
 * @return std::string ISO 8601 格式的时间戳字符串
 *         - 格式：YYYY-MM-DDTHH:MM:SS.mmmZ
 *         - 示例：2024-01-15T14:30:45.123Z
 *         - 时区：UTC（使用 'Z' 标识）
 *         - 精度：毫秒级（3 位小数）
 * 
 * @note 
 * - 使用 std::gmtime() 获取 UTC 时间，不受系统本地时区影响
 * - 毫秒部分通过 std::chrono 高精度时钟计算，确保准确性
 * - 符合 ISO 8601 国际标准，适用于 SAPIENT 协议的时间戳字段
 * 
 * @example
 * std::string timestamp = getCurrentTimeISO8601();
 * // 输出示例：2024-01-15T14:30:45.123Z
 */
std::string getCurrentTimeISO8601() 
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 转换为 time_t 类型，以便后续格式化
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // 使用 std::gmtime 将 time_t 转为 tm 格式（UTC 时间）
    std::tm utc_tm = *std::gmtime(&now_time_t);

    // 使用字符串流格式化时间
    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S");

    // 获取当前时间的毫秒部分
    auto duration = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;

    // 将毫秒部分添加到字符串
    oss << '.' << std::setw(3) << std::setfill('0') << milliseconds.count();

    // 添加 Z 以指示 UTC 时间
    oss << 'Z';

    return oss.str();
}

/*从完整版本字符串中提取版本号部分（可选函数）*/
static std::string extractVersionNumber(const std::string& full_version) 
{
    // 支持多种常见格式：
    // 1) "STP120-GE-FW-V10.01.05.34-STD" -> "10.01.05.34"
    // 2) "SFL100_GE_FW_V10.01.06.02_STD" -> "10.01.06.02"
    // 3) "ACUR100-T6-V00.00.00" -> "00.00.00"
    // 4) "ACUR101-V10.01.05.34" -> "10.01.05.34"
    
    // 先尝试查找 "_V"（下划线分隔）
    size_t v_pos = full_version.find("_V");
    size_t sep_len = 2;
    
    // 如果没找到，尝试查找 "-V"（横线分隔）
    if (v_pos == std::string::npos) {
        v_pos = full_version.find("-V");
        sep_len = 2;
    }
    
    // 如果还是没找到，尝试查找 "V"（直接查找，可能是 "V10.01.05.34" 格式）
    if (v_pos == std::string::npos) {
        v_pos = full_version.find("V");
        sep_len = 1;
    }
    
    if (v_pos != std::string::npos) {
        size_t start = v_pos + sep_len; // 跳过 "_V"、"-V" 或 "V"
        
        // 结束分隔符可能是 "_" 或 "-"，如果都没有则到字符串末尾
        size_t end_underscore = full_version.find("_", start);
        size_t end_dash = full_version.find("-", start);
        size_t end = std::min(
            end_underscore == std::string::npos ? full_version.size() : end_underscore,
            end_dash == std::string::npos ? full_version.size() : end_dash
        );
        
        std::string version = full_version.substr(start, end - start);
        
        // 验证提取的版本号格式（应该是数字和点号组成，如 "10.01.05.34" 或 "00.00.00"）
        if (!version.empty() && version.find_first_not_of("0123456789.") == std::string::npos) {
            return version;
        }
    }
    
    // 如果格式不匹配，返回空字符串（让调用者使用默认值）
    return "";
}

/*根据设备经纬度，计算出经度zone*/
static std::string calculateUTMZone(double longitude, double latitude)
{
    // Ensure the longitude is between -180 and 180
    if (longitude < -180.0 || longitude > 180.0) {
        std::cerr << "Error: Longitude must be between -180 and 180 degrees." << std::endl;
        return ""; // Return empty string as error indicator
    }
    
    // Calculate the UTM zone number
    int zoneNumber = static_cast<int>(std::floor((longitude + 180.0) / 6.0)) + 1;
    
    // Determine hemisphere (N/S)
    char hemisphere = (latitude >= 0) ? 'N' : 'S';
    
    // Create the UTM zone string
    return std::to_string(zoneNumber) + hemisphere;
}

std::string getUTMZone(void)
{
    GNSS_coordinate_t coordinate;
    auto_hunt_param_get_GNSS(&coordinate);
    std::cout << "coordinate.longitude = " <<coordinate.longitude << std::endl;
    std::cout << "coordinate.latitude = " <<coordinate.latitude << std::endl;

    coordinate.longitude = coordinate.longitude /1e7;
    coordinate.latitude = coordinate.latitude / 1e7;

    if((0 == coordinate.longitude) && (0 == coordinate.latitude)){
        coordinate.longitude = 114.0579;
        coordinate.latitude = 22.5431;
    }
    return calculateUTMZone(coordinate.longitude, coordinate.latitude);
}

// 前置声明：可复用的构建函数，在文件后面定义（C++ 链接）
int sapient_build_registration(std::string &out_serialized, std::string &out_json);

extern "C" {
int sapient_register(void) 
{
    // 简化：在构建函数内部完成整个 SapientMessage wrapper 的构造和序列化
    // sapient_build_registration 会返回序列化的 SapientMessage（二进制和 JSON）
    std::string out_bin;
    std::string out_json;
    int ret = sapient_build_registration(out_bin, out_json);
    if (ret != 0) {
        std::cerr << "sapient_build_registration failed" << std::endl;
        return -1;
    }

    // 打印 JSON 以便人工查看（保留原有行为）
    std::cout << "Serialized JSON output (SapientMessage wrapper): " << std::endl;
    std::cout << out_json << std::endl;

    return 0;
}
}

// ---------------------------------------------------------------------------
// 可复用的构建函数
// 构造 SkyRegistrationMessage，返回二进制 protobuf 到 out_serialized
// 和格式化的 JSON 表示到 out_json。成功返回 0，失败返回 -1。
// 该函数为 C++ 链接，可以从其他 C++ 转换单元调用
// （例如在 `sapient_tcp.cpp` 中实现的 TCP 发送器）。
int sapient_build_registration(std::string &out_serialized, std::string &out_json)
{
    sapient_msg::bsi_flex_335_v2_0::SkyRegistrationMessage pbmsg;

    getSn();

    /*timestamp*/
    std::string timestamp = getCurrentTimeISO8601();
    pbmsg.set_timestamp(timestamp);

    /*nodeId - 使用 UUID v5 基于设备序列号生成（确定性，符合 UUID 格式）*/
    std::string node_id = generateNodeID();
    pbmsg.set_nodeid(node_id);

    /*registration*/
    auto *reg = pbmsg.mutable_registration();

    auto *node_def = reg->add_node_definition();
    node_def->set_node_type(sapient_msg::bsi_flex_335_v2_0::Registration_NodeType_NODE_TYPE_RADAR);
    // node_def->add_node_sub_type("UAV_RADIO_DETECTION");
    // node_def->add_node_sub_type("SIGNAL_MODULATION_ANALYSIS");
    // node_def->add_node_sub_type("RANGE_BEARING_POSITIONING");

    reg->set_icd_version("BSI Flex 335 v2.0");
    reg->set_name(sapient_product_display_name());
    reg->set_short_name(sapient_product_short_name());

    /*设备探测能力*/
    auto *capacity = reg->add_capabilities();
    capacity->set_category("Platform");
    capacity->set_type("Type");
    capacity->set_value("Installation");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("Technology");
    capacity->set_value("AESA_FMCW");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("MinFrequency");
    capacity->set_value("24050");
    capacity->set_units("MHz");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("MaxFrequency");
    capacity->set_value("24250");
    capacity->set_units("MHz");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("Bandwidth");
    capacity->set_value("50");
    capacity->set_units("MHz");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("MaxRange");
    capacity->set_value("4000");
    capacity->set_units("m");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("AzimuthFOV");
    capacity->set_value("100");
    capacity->set_units("deg");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("ElevationFOV");
    capacity->set_value("45");
    capacity->set_units("deg");

    capacity = reg->add_capabilities();
    capacity->set_category("Radar");
    capacity->set_type("MaxTargets");
    capacity->set_value("200");
    
    /*设置探测的时间100ms上报一次数据 */
    auto *statusdef = reg->mutable_status_definition();
    auto *duration = statusdef->mutable_status_interval();
    duration->set_value(5.0f);
    duration->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_SECONDS);

    /*设置设备定位坐标系*/
    auto *locationdef = statusdef->mutable_location_definition();
    locationdef->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    locationdef->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);
    //locationdef->set_zone(getUTMZone());  //根据经纬度设置zone

    // /*设置覆盖区域的坐标系 */
    // auto *coveragedef = statusdef->mutable_coverage_definition();
    // coveragedef->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    // coveragedef->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_E);
    // //coveragedef->set_zone(getUTMZone());

    // /*设置遮挡区域的坐标系 */
    // auto *obscurationdef = statusdef->mutable_obscuration_definition();
    // obscurationdef->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    // obscurationdef->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_E);
    // //obscurationdef->set_zone(getUTMZone());

    /*设置视场范围区域（field_of_view）的坐标系 */
    auto *fieldofviewdef = statusdef->mutable_field_of_view_definition();
    fieldofviewdef->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    fieldofviewdef->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);

    // /*Additional heartbeat report definitions 心跳上报定义*/
    // auto *StatusRep = statusdef->add_status_report();
    // StatusRep->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_StatusReportCategory_STATUS_REPORT_CATEGORY_STATUS); //设备状态
    // StatusRep->set_type("Platform");
    // //StatusRep->set_units("Stationary");
    // StatusRep->set_on_change(false);  // 布尔类型，不是字符串

    // OTM Mode (Motion Sensitivity)
    auto *StatusRep = statusdef->add_status_report();
    StatusRep->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_StatusReportCategory_STATUS_REPORT_CATEGORY_STATUS);
    StatusRep->set_type("MOTION_SENSITIVITY");
    StatusRep->set_on_change(false);

    // 滤波等级 (Filter Level / Clutter)、气象杂波抑制 (Weather Clutter Filter)
    StatusRep = statusdef->add_status_report();
    StatusRep->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_StatusReportCategory_STATUS_REPORT_CATEGORY_STATUS);
    StatusRep->set_type("CLUTTER");
    StatusRep->set_on_change(false);

    // 故障信息 (Internal Fault)
    StatusRep = statusdef->add_status_report();
    StatusRep->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_StatusReportCategory_STATUS_REPORT_CATEGORY_STATUS);
    StatusRep->set_type("INTERNAL_FAULT");
    StatusRep->set_on_change(false);
    
    /*mode_define*/
    /*设置模式*/
    auto *modedef = reg->add_mode_definition();
    modedef->set_mode_name("Standby");
    modedef->set_mode_type(sapient_msg::bsi_flex_335_v2_0::Registration_ModeType_MODE_TYPE_DEFAULT);
    modedef->set_mode_description("The node is available for tasking");
    auto *duration_time = modedef->mutable_settle_time();
    duration_time->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_MILLISECONDS);
    duration_time->set_value(1000);  

    auto *taskdef = modedef->mutable_task();
    taskdef->set_concurrent_tasks(10);
    auto *regiondef = taskdef->mutable_region_definition();
    regiondef->add_region_type(sapient_msg::bsi_flex_335_v2_0::Registration_RegionType_REGION_TYPE_AREA_OF_INTEREST);
    // auto *durationsettletime = regiondef->mutable_settle_time();
    // durationsettletime->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_SECONDS);
    // durationsettletime->set_value(5.0);
    auto *regionarea = regiondef->add_region_area();
    regionarea->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    regionarea->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);

    /*Command 定义*/
    auto *task_command = taskdef->add_command();
    task_command->set_units("Normal_Detection");
    auto *completion_time = task_command->mutable_completion_time();
    completion_time->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_SECONDS);
    completion_time->set_value(1.0);
    task_command->set_type(sapient_msg::bsi_flex_335_v2_0::Registration_CommandType_COMMAND_TYPE_MODE_CHANGE);

    /*mode_define*/
    /*设置模式*/
    auto *modedef2 = reg->add_mode_definition();
    modedef2->set_mode_name("Normal_Detection");
    modedef2->set_mode_type(sapient_msg::bsi_flex_335_v2_0::Registration_ModeType_MODE_TYPE_DEFAULT);
    //modedef2->set_mode_description("The node is detecting targets");
    auto *duration_time2 = modedef2->mutable_settle_time();
    duration_time2->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_MILLISECONDS);
    duration_time2->set_value(1000);  

    /*延时设置*/
    auto *duration_latency2 = modedef2->mutable_maximum_latency();
    duration_latency2->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_MILLISECONDS);
    duration_latency2->set_value(3000);

    /*扫描设置 */
    modedef2->set_scan_type(sapient_msg::bsi_flex_335_v2_0::Registration_ScanType_SCAN_TYPE_FIXED);  //侦测固定区域
    modedef2->set_tracking_type(sapient_msg::bsi_flex_335_v2_0::Registration_TrackingType_TRACKING_TYPE_TRACK); //追踪

    /*不确定表示的是什么意思，先这么填上*/
    //auto *duration1 = modedef2->mutable_duration();
    //duration1->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_MILLISECONDS);
    //duration1->set_value(100);

    /*额外的参数设置*/
    // auto *modepara2 = modedef2->add_mode_parameter();
    // modepara2->set_type("SelfAdaptation");
    // modepara2->set_value("Range");
    
    /*侦测定义*/
    auto *detectdef2 = modedef2->add_detection_definition();
    auto *locationtype2 = detectdef2->mutable_location_type();
    locationtype2->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    locationtype2->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);
    //locationtype2->set_zone(getUTMZone());  // 使用经纬度时不需要设置 UTM zone

    /*侦测性能定义, 不知道怎么填，先暂时这样填*/
    // auto *detectperf2 = detectdef2->add_detection_performance();
    // detectperf2->set_type("FAR");
    // detectperf2->set_units("Per Second");
    // detectperf2->set_unit_value("1");
    // detectperf2->set_variation_type("Linear with range");

    // 侦测数据上报设置（object_info 额外字段声明）
    // 注意：只声明实际上报的、且不在标准字段中的补充信息
    
    // 1. RCS（雷达散射截面）- 目标反射强度
    auto *detectrepo2 = detectdef2->add_detection_report();
    detectrepo2->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_DetectionReportCategory_DETECTION_REPORT_CATEGORY_OBJECT); 
    detectrepo2->set_type("RCS");
    detectrepo2->set_units("dBsm");
    //detectrepo2->set_on_change(false);

    // 2. 对地速度（绝对速度）- 与径向速度不同
    detectrepo2 = detectdef2->add_detection_report();
    detectrepo2->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_DetectionReportCategory_DETECTION_REPORT_CATEGORY_OBJECT); 
    detectrepo2->set_type("absVel");
    detectrepo2->set_units("m/s");
    //detectrepo2->set_on_change(false);

    // 3. 航向角 - 目标运动方向
    detectrepo2 = detectdef2->add_detection_report();
    detectrepo2->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_DetectionReportCategory_DETECTION_REPORT_CATEGORY_OBJECT); 
    detectrepo2->set_type("heading");
    detectrepo2->set_units("deg");
    //detectrepo2->set_on_change(false);

    // 4. 跟踪时长 - 体现跟踪稳定性
    detectrepo2 = detectdef2->add_detection_report();
    detectrepo2->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_DetectionReportCategory_DETECTION_REPORT_CATEGORY_OBJECT); 
    detectrepo2->set_type("trackDuration");
    detectrepo2->set_units("s");
    //detectrepo2->set_on_change(false);

    // 5. 跟踪类型 - TWS（搜索跟踪）或 TAS（目标跟踪）
    detectrepo2 = detectdef2->add_detection_report();
    detectrepo2->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_DetectionReportCategory_DETECTION_REPORT_CATEGORY_OBJECT); 
    detectrepo2->set_type("trackType");
    detectrepo2->set_units("TWS, TAS");  // 有效值列表（必填字段）
    //detectrepo2->set_on_change(false);

    // 6. 航迹状态 - Confirmed（稳态）或 Tentative（暂态）
    detectrepo2 = detectdef2->add_detection_report();
    detectrepo2->set_category(sapient_msg::bsi_flex_335_v2_0::Registration_DetectionReportCategory_DETECTION_REPORT_CATEGORY_OBJECT); 
    detectrepo2->set_type("trackState");
    detectrepo2->set_units("Confirmed, Tentative");  // 有效值列表（必填字段）
    //detectrepo2->set_on_change(false);

    //侦测类定义
    auto *detectclassdef2 = detectdef2->add_detection_class_definition();
    detectclassdef2->set_confidence_definition(sapient_msg::bsi_flex_335_v2_0::Registration_ConfidenceDefinition_CONFIDENCE_DEFINITION_SINGLE_CLASS);
    
    //侦测类性能设置
    // auto *classperf2 = detectclassdef2->add_class_performance();
    // classperf2->set_type("FAR");
    // classperf2->set_units("Per Second");
    // classperf2->set_unit_value("1");
    // classperf2->set_variation_type("Linear with range");

    //类定义：Air vehicle（无人机）
    auto *classdef2 = detectclassdef2->add_class_definition();
    classdef2->set_type("Air vehicle");
    // classdef2->set_units("0.9");
    auto *subclass2 = classdef2->add_sub_class();
    subclass2->set_type("UAV rotary wing");
    // subclass2->set_units("1");
    subclass2->set_level(1);
    // auto *subclass1_2 = subclass2->add_sub_class();
    // subclass1_2->set_type("Commercial");
    // // subclass1_2->set_units("1"); 
    // subclass1_2->set_level(2);

    //类定义：Human（单兵）
    auto *classdef_human = detectclassdef2->add_class_definition();
    classdef_human->set_type("Human");

    //类定义：Land vehicle（车辆）
    auto *classdef_land = detectclassdef2->add_class_definition();
    classdef_land->set_type("Land vehicle");

    //类定义：Animal > Bird（鸟类）
    auto *classdef_animal = detectclassdef2->add_class_definition();
    classdef_animal->set_type("Animal");
    {
        auto *subclass_animal = classdef_animal->add_sub_class();
        subclass_animal->set_type("Bird");
        subclass_animal->set_level(1);
    }

    //类定义：Unknown（未识别目标）
    auto *classdef_unknown = detectclassdef2->add_class_definition();
    classdef_unknown->set_type("Unknown");

    //类定义：Other（其他目标）
    auto *classdef_other = detectclassdef2->add_class_definition();
    classdef_other->set_type("Other");

    // auto *taxonomydef2 = detectclassdef2->add_taxonomy_dock_definition();
    // taxonomydef2->set_dock_class_namespace("dock");
    // taxonomydef2->set_dock_class("dock_class");
    // auto *extendsubclass2 = taxonomydef2->add_extension_subclass();
    // extendsubclass2->set_subclass_namespace("extension class");
    // extendsubclass2->set_subclass_name("subclass");
    // extendsubclass2->set_units("1");

    // Behaviour 定义：DetectionReport.behaviour.type 建议只能使用 Registration 中声明过的值，
    // 否则部分对端工具会降级显示为 "other"。
    {
        auto *behaviourdef2 = detectdef2->add_behaviour_definition();
        behaviourdef2->set_type("Active");
    }
    {
        auto *behaviourdef2 = detectdef2->add_behaviour_definition();
        behaviourdef2->set_type("Passive");
    }

    auto *velocity2 = detectdef2->mutable_velocity_type();
    auto *enuvelocityunits2 = velocity2->mutable_enu_velocity_units();
    enuvelocityunits2->set_east_north_rate_units(sapient_msg::bsi_flex_335_v2_0::SPEED_UNITS_MS);
    enuvelocityunits2->set_up_rate_units(sapient_msg::bsi_flex_335_v2_0::SPEED_UNITS_MS);

    velocity2->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);
    // velocity2->set_zone(getUTMZone());  // 使用经纬度时不需要设置 UTM zone

    auto *geometric2 = detectdef2->mutable_geometric_error();
    geometric2->set_type("Standard Deviation");
    geometric2->set_units("meters");
    geometric2->set_variation_type("Linear with range");
    // auto *perform2 = geometric2->add_performance_value();
    // perform2->set_type("speed");
    // perform2->set_units("meters");
    // perform2->set_unit_value("1");
    // perform2->set_variation_type("Linear");

    auto *taskdef2 = modedef2->mutable_task();
    taskdef2->set_concurrent_tasks(10);
    auto *regiondef2 = taskdef2->mutable_region_definition();
    regiondef2->add_region_type(sapient_msg::bsi_flex_335_v2_0::Registration_RegionType_REGION_TYPE_AREA_OF_INTEREST);
    // auto *durationsettletime2 = regiondef2->mutable_settle_time();
    // durationsettletime2->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_SECONDS);
    // durationsettletime2->set_value(5.0);
    auto *regionarea2 = regiondef2->add_region_area();
    regionarea2->set_location_units(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    regionarea2->set_location_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);
    // regionarea2->set_zone(getUTMZone());  // 使用经纬度时不需要设置 UTM zone

    // auto *classfilterdef2 = regiondef2->add_class_filter_definition();
    // auto *filterpara2 = classfilterdef2->add_filter_parameter();
    // filterpara2->set_parameter("no");
    // filterpara2->add_operators(sapient_msg::bsi_flex_335_v2_0::OPERATOR_LESS_THAN);
    // classfilterdef2->set_type("DroneType");
 
    
    // auto *behaviourfilterdef2 = regiondef2->add_behaviour_filter_definition();
    // auto *behaviour_fileter_para2 = behaviourfilterdef2->add_filter_parameter();
    // behaviour_fileter_para2->set_parameter("no");
    // behaviour_fileter_para2->add_operators(sapient_msg::bsi_flex_335_v2_0::OPERATOR_LESS_THAN);
    // behaviourfilterdef2->set_type("no");

    auto *task_command2 = taskdef2->add_command();
    task_command2->set_units("Standby");
    auto *completion_time2 = task_command2->mutable_completion_time();
    completion_time2->set_units(sapient_msg::bsi_flex_335_v2_0::Registration_TimeUnits_TIME_UNITS_SECONDS);
    completion_time2->set_value(1.0);
    task_command2->set_type(sapient_msg::bsi_flex_335_v2_0::Registration_CommandType_COMMAND_TYPE_MODE_CHANGE);

    // 依赖节点：如果设备有依赖的其他节点，在这里添加
    // 目前 STP120 是独立设备，不需要依赖节点
    // const char* node1 = "cc6203a8-2ab8-4a8b-92b7-195f31b9d43e";
    // const char* node2 = "59c5e9c5-4954-4b5d-8da9-0027607669a7";
    // reg->add_dependent_nodes(node1, strlen(node1));
    // reg->add_dependent_nodes(node2, strlen(node2));

    // 设置上报区域：使用设备实际位置（如果可用）
    // auto *reportregion = reg->add_reporting_region();
    // auto *locationlist = reportregion->mutable_location_list();
    // auto *plocation = locationlist->add_locations();
    // {
    //     GNSS_coordinate_t device_gnss;
    //     auto_hunt_param_get_GNSS(&device_gnss);
        
    //     // 如果设备有有效坐标，使用实际位置；否则使用默认值（0,0）
    //     if (device_gnss.longitude != 0 || device_gnss.latitude != 0) {
    //         plocation->set_x(device_gnss.longitude / 1e7);  // 经度（度）
    //         plocation->set_y(device_gnss.latitude / 1e7);   // 纬度（度）
    //         plocation->set_z(device_gnss.altitude / 10.0);  // 海拔（米，原始单位是0.1米）
    //     } else {
    //         plocation->set_x(0.0);
    //         plocation->set_y(0.0);
    //         plocation->set_z(0.0);
    //     }
    // }
    // plocation->set_x_error(10.0);  // 经度误差（米）
    // plocation->set_y_error(10.0);  // 纬度误差（米）
    // plocation->set_z_error(5.0);   // 海拔误差（米）
    // plocation->set_coordinate_system(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
    // plocation->set_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_E);  // 统一使用 WGS84_E

    auto *configdata = reg->add_config_data();
    configdata->set_manufacturer("Skyfend");
    configdata->set_model(sapient_product_short_name());
    
    configdata->set_serial_number(g_sn);
    configdata->set_hardware_version("1.0.0.0");
    // 使用运行时获取的版本号，提取版本号部分
    const char *full_version = get_embed_software_ps_version_string();
    if (full_version && strlen(full_version) > 0) {
        std::string extracted_version = extractVersionNumber(std::string(full_version));
        // 如果提取失败（返回原字符串且格式不匹配），使用默认值
        if (extracted_version.empty() || 
            (extracted_version == full_version && extracted_version.find("V") == std::string::npos)) {
            std::cerr << "Warning: Failed to extract version from '" << full_version 
                      << "', using default version" << std::endl;
            configdata->set_software_version("1.0.0.0");  // 默认版本号
        } else {
            std::cout << "Extracted software version: '" << extracted_version 
                      << "' from '" << full_version << "'" << std::endl;
            configdata->set_software_version(extracted_version);
        }
    } else {
        std::cerr << "Warning: get_embed_software_ps_version_string() returned NULL or empty, using default version" << std::endl;
        configdata->set_software_version("1.0.0.0");  // 默认版本号
    }
    // auto *configdatasub = configdata->add_sub_components();
    // configdatasub->set_manufacturer("skyfend");
    // configdatasub->set_model("stp120");
    // configdatasub->set_serial_number(g_sn);
    // configdatasub->set_hardware_version("1.0.0");
    // configdatasub->set_software_version("2.0.0");

    // 构造 SapientMessage wrapper，并将 pbmsg 的 registration 放入 oneof 中，
    // 同时将 node_id 和 timestamp 放到 wrapper 顶层（避免对端解析错误）。
    sapient_msg::bsi_flex_335_v2_0::SapientMessage wrapper;
    if (pbmsg.nodeid().size() > 0) {
        wrapper.set_node_id(pbmsg.nodeid());
    }
    // 顶层 timestamp 使用 google::protobuf::Timestamp 类型，
    // 直接用当前系统时间填充 seconds/nanos，避免使用字符串赋值导致的类型不匹配。
    {
        auto now = std::chrono::system_clock::now();
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        auto nanos_total = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        int nanos = static_cast<int>(nanos_total - secs * 1000000000LL);
        google::protobuf::Timestamp* ts = wrapper.mutable_timestamp();
        ts->set_seconds(static_cast<long long>(secs));
        ts->set_nanos(nanos);
    }
    // 把 registration 字段拷入 wrapper 的 registration oneof
    wrapper.mutable_registration()->CopyFrom(pbmsg.registration());

    // 序列化 wrapper 到二进制
    if (!wrapper.SerializeToString(&out_serialized)) {
        std::cerr << "Failed to serialize SapientMessage wrapper in builder" << std::endl;
        return -1;
    }

    // 序列化 wrapper 到 JSON（用于调试/日志）
    google::protobuf::util::JsonOptions options; 
    options.add_whitespace = true;  // 格式化输出增加可读性
    auto status = google::protobuf::util::MessageToJsonString(wrapper, &out_json, options);
    if (!status.ok()) {
        std::cerr << "Failed to convert wrapper message to JSON format: " << status.ToString() << std::endl;
        return -1;
    }

    return 0;
}