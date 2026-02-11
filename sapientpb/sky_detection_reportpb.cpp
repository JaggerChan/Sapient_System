

#include <iostream>
#include <string>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <map>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/timestamp.pb.h>
#include "../sapient/sky_detection_report.pb.h"
#include "../sapient/sapient_message.pb.h"
#include "../../inc/invaild_value.h"
#include "../../common/nanopb/radar.pb.h"

extern "C" {
#include "adapter/auto_hunt_param_adapter.h"
#include "adapter/radar_state_adapter.h"  // 获取雷达状态（包括航向角）
}

#include "sapient_tcp.h"
#include "sky_task_handler.h"
#include "sapient_nodeid.h"

extern std::string g_sn;
extern std::string getCurrentTimeISO8601();
std::string getUTMZone(void);

// Base64 编码表
static const char base64_chars[] = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

// Base64 编码函数（保留用于未来使用）
static std::string __attribute__((unused)) base64_encode(const unsigned char* data, size_t len) {
    std::string ret;
    int i = 0;
    unsigned char array_3[3];
    unsigned char array_4[4];

    while (len--) {
        array_3[i++] = *(data++);
        if (i == 3) {
            array_4[0] = (array_3[0] & 0xfc) >> 2;
            array_4[1] = ((array_3[0] & 0x03) << 4) + ((array_3[1] & 0xf0) >> 4);
            array_4[2] = ((array_3[1] & 0x0f) << 2) + ((array_3[2] & 0xc0) >> 6);
            array_4[3] = array_3[2] & 0x3f;

            for(i = 0; i < 4; i++)
                ret += base64_chars[array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for(int j = i; j < 3; j++)
            array_3[j] = '\0';

        array_4[0] = (array_3[0] & 0xfc) >> 2;
        array_4[1] = ((array_3[0] & 0x03) << 4) + ((array_3[1] & 0xf0) >> 4);
        array_4[2] = ((array_3[1] & 0x0f) << 2) + ((array_3[2] & 0xc0) >> 6);

        for (int j = 0; j < i + 1; j++)
            ret += base64_chars[array_4[j]];

        while(i++ < 3)
            ret += '=';
    }

    return ret;
}

extern "C" {

    #include <stdio.h>
    #include <stdlib.h>
    #include <time.h>
    #include <string.h>
    #include <stdint.h>


    // Crockford's Base32 字母表
    const char* base32_chars = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

    // 将无符号整数编码为 Base32 表示形式
    static void encode_base32_u64(uint64_t value, char *dest, int count)
    {
        for (int i = count - 1; i >= 0; --i) {
            dest[i] = base32_chars[value % 32];
            value /= 32;
        }
    }

    // 生成 ULID（唯一性、单调递增的标识符）
    // 修复：使用毫秒级时间戳 + 线程安全的随机数生成器，避免高频上报时 ULID 重复
    void generate_ulid(char *ulid) 
    {
        // 使用 chrono 获取真正的毫秒级时间戳
        auto now = std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        uint64_t timestamp_ms = static_cast<uint64_t>(millis);

        // 编码时间戳（占用前10个字符）
        encode_base32_u64(timestamp_ms, ulid, 10);

        // 添加随机部分（占用后16个字符）
        // 使用 thread_local 随机数生成器，每个线程只初始化一次，避免同秒重复序列
        static thread_local std::mt19937 rng([]() {
            std::random_device rd;
            auto seed = rd();
            if (seed == 0) {
                // 如果 random_device 失败，使用时间作为后备种子
                seed = static_cast<unsigned int>(
                    std::chrono::high_resolution_clock::now().time_since_epoch().count());
            }
            return seed;
        }());
        std::uniform_int_distribution<int> dist(0, 31);
        
        for (int i = 10; i < 26; ++i) {
            ulid[i] = base32_chars[dist(rng)];
        }

        // 字符串以空字符结尾
        ulid[26] = '\0';
    }
}

// 新实现：基于 RadarTrackItem 构建 DetectionReport（应用层数据源）
static int sapient_build_detection_report_from_track_item(
    std::string &out_serialized,
    std::string &out_json,
    const RadarTrackItem *track_item)
{
    if (!track_item) {
        std::cerr << "Error: track_item is null" << std::endl;
        return -1;
    }

    char ulid[27] = {0};
    sapient_msg::bsi_flex_335_v2_0::SkyDetectionReportMessage detectrepo;

    // 注意：不要在每次构建报文时调用 srand(time(NULL))，否则同一秒内：
    // 1) 随机数种子相同 → rand() 序列重复 → ULID 的随机部分重复
    // 2) time(NULL) 秒级精度 → ULID 时间戳部分也相同
    // 结果：同一秒内生成的所有 ULID 完全重复，导致对端代理去重/丢弃
    // 现在 generate_ulid() 使用毫秒级时间戳 + thread_local 随机数生成器，无需 srand()

    // 填充报文头部
    std::string timestamp = getCurrentTimeISO8601();
    detectrepo.set_timestamp(timestamp);
    
    // 使用统一的 NodeID 生成接口
    std::string node_id = generateNodeID();
    detectrepo.set_nodeid(node_id);

    // 填充 detection report 内容
    auto *detectionreport = detectrepo.mutable_detectionreport();
    generate_ulid(ulid);
    detectionreport->set_report_id(ulid);

    // object_id 生成：基于 track ID（持久化）
    static std::map<uint32_t, std::string> track_id_to_objectid;
    std::string object_id;
    auto it = track_id_to_objectid.find(track_item->id);
    if (it != track_id_to_objectid.end()) {
            object_id = it->second;
    } else {
        generate_ulid(ulid);
        object_id = ulid;
        track_id_to_objectid[track_item->id] = object_id;
    }
    detectionreport->set_object_id(object_id);

    // task_id：仅在存在有效任务 ID 时设置
    std::string current_task_id = sapient_get_current_task_id();
    if (!current_task_id.empty()) {
        detectionreport->set_task_id(current_task_id);
    }

    // 状态：有 RadarTrackItem 数据就设置为 "detected"
    // （类似 STP120 的 "if (pdrone)" 逻辑，能执行到这里说明 track_item 不为空）
        detectionreport->set_state("detected");

    // ======================== 获取雷达状态数据（用于坐标转换） ========================
    RadarState radar_state;
    memset(&radar_state, 0, sizeof(RadarState));
    double radar_heading = 0.0;  // 雷达平台航向角（相对于正北）
    
    int ret_state = get_radar_state(&radar_state);
    if (ret_state == 0 && radar_state.has_attitude && radar_state.attitude.has_heading) {
        radar_heading = radar_state.attitude.heading;
    } else {
        // 如果获取失败，使用默认值 0（假设雷达朝北）
        radar_heading = 0.0;
    }
    // ================================================================================

    // 位置：优先使用 GPS 坐标（如果有），否则使用 RangeBearing
    if (track_item->longitude != 0.0f || track_item->latitude != 0.0f) {
        // 使用 Location (经纬度)
        auto *lc = detectionreport->mutable_location();
        lc->set_x(track_item->longitude);  // 经度（度）
        lc->set_y(track_item->latitude);  // 纬度（度）
        
        // 海拔：有效范围 -10000 ~ 10000m
        if (track_item->altitude >= -10000.0f && track_item->altitude <= 10000.0f) {
            lc->set_z(track_item->altitude);  // 高度（米）
        }
        
        // 误差设置（根据坐标方差估算）
        const double METERS_PER_DEGREE = 111000.0;
        const double ERROR_METERS = 6.0;
        double error_deg = round((ERROR_METERS / METERS_PER_DEGREE) * 100000.0) / 100000.0;
        
        lc->set_x_error(error_deg);
        lc->set_y_error(error_deg);
        //lc->set_z_error(ERROR_METERS);
        lc->set_coordinate_system(sapient_msg::bsi_flex_335_v2_0::LOCATION_COORDINATE_SYSTEM_LAT_LNG_DEG_M);
        lc->set_datum(sapient_msg::bsi_flex_335_v2_0::LOCATION_DATUM_WGS84_G);
    } else {
        // 使用 RangeBearing (方位角/仰角/距离)
        auto *rb = detectionreport->mutable_range_bearing();
        
        // 方位角（度）：SAPIENT 要求方位角相对于正北
        // track_item->azimuth 是相对于雷达的角度（-60° ~ 60°）
        // 需要转换为相对于正北的角度
        if (track_item->azimuth >= -60.0f && track_item->azimuth <= 60.0f) {
            // 转换公式：相对于正北的方位角 = 相对于雷达的方位角 + 雷达航向角
            double azimuth_relative_to_north = track_item->azimuth + radar_heading;
            
            // 归一化到 [0, 360) 范围
            while (azimuth_relative_to_north < 0.0) {
                azimuth_relative_to_north += 360.0;
            }
            while (azimuth_relative_to_north >= 360.0) {
                azimuth_relative_to_north -= 360.0;
            }
            
            rb->set_azimuth(azimuth_relative_to_north);
            rb->set_azimuth_error(1.0);  // 方位角误差 1°
        }
        
        // 仰角（度）：有效范围 -40° ~ 40°
        if (track_item->elevation >= -40.0f && track_item->elevation <= 40.0f) {
            rb->set_elevation(track_item->elevation);
            rb->set_elevation_error(1.0);  // 仰角误差 1°
        }
        
        // 距离（米）：有效范围 0 ~ 6000m
        if (track_item->range > 0.0f && track_item->range <= 6000.0f) {
            rb->set_range(track_item->range);
            // 距离误差：使用固定值 10m（与 STP120 一致）
            // 注意：x_variance 是 x 方向标准差，不能直接用于径向距离误差
            rb->set_range_error(10.0);
        }
        
        rb->set_coordinate_system(sapient_msg::bsi_flex_335_v2_0::RANGE_BEARING_COORDINATE_SYSTEM_DEGREES_M);
        rb->set_datum(sapient_msg::bsi_flex_335_v2_0::RANGE_BEARING_DATUM_TRUE);
    }

    // 检测置信度：使用目标存在概率
    float confidence = track_item->existingProb / 100.0f;
    if (confidence > 1.0f) confidence = 1.0f;
    if (confidence < 0.0f) confidence = 0.0f;
    detectionreport->set_detection_confidence(confidence);

    // 对象信息（object_info）
    // 1. 存在概率
    // auto *info_exist_prob = detectionreport->add_object_info();
    // info_exist_prob->set_type("existingProb");
    // char exist_prob_buf[32];
    // snprintf(exist_prob_buf, sizeof(exist_prob_buf), "%u%%", track_item->existingProb);
    // info_exist_prob->set_value(exist_prob_buf);
    
    // 4. 距离：有效范围 0 ~ 6000m
    if (track_item->range > 0.0f && track_item->range <= 6000.0f) {
        auto *info_range = detectionreport->add_object_info();
        info_range->set_type("range");
        char range_buf[32];
        snprintf(range_buf, sizeof(range_buf), "%.2fm", track_item->range);
        info_range->set_value(range_buf);
    }
    
    // 5. 方位角：有效范围 -60° ~ 60°
    if (track_item->azimuth >= -60.0f && track_item->azimuth <= 60.0f) {
        auto *info_azimuth = detectionreport->add_object_info();
        info_azimuth->set_type("azimuth");
        char azimuth_buf[32];
        snprintf(azimuth_buf, sizeof(azimuth_buf), "%.2f°", track_item->azimuth);
        info_azimuth->set_value(azimuth_buf);
    }
    
    // 6. 仰角：有效范围 -40° ~ 40°
    if (track_item->elevation >= -40.0f && track_item->elevation <= 40.0f) {
        auto *info_elevation = detectionreport->add_object_info();
        info_elevation->set_type("elevation");
        char elevation_buf[32];
        snprintf(elevation_buf, sizeof(elevation_buf), "%.2f°", track_item->elevation);
        info_elevation->set_value(elevation_buf);
    }
    
    // 7. 径向速度：有效范围 -50 ~ 50 m/s
    if (track_item->velocity >= -50.0f && track_item->velocity <= 50.0f) {
        auto *info_velocity = detectionreport->add_object_info();
        info_velocity->set_type("velocity");
        char velocity_buf[32];
        snprintf(velocity_buf, sizeof(velocity_buf), "%.2fm/s", track_item->velocity);
        info_velocity->set_value(velocity_buf);
    }
    
    // 8. 绝对速度（对地速度）：有效范围 0 ~ 100 m/s
    if (track_item->absVel >= 0.0f && track_item->absVel <= 100.0f) {
        auto *info_abs_vel = detectionreport->add_object_info();
        info_abs_vel->set_type("absVel");
        char abs_vel_buf[32];
        snprintf(abs_vel_buf, sizeof(abs_vel_buf), "%.2fm/s", track_item->absVel);
        info_abs_vel->set_value(abs_vel_buf);
    }
    
    // 9. RCS（雷达散射截面）
    // 注意：track_item->RCS 的计算公式为 (mag/64.0 - 200.0)，单位为 dBsm
    // 典型范围：小型无人机 -40~0 dBsm，大型目标 0~+40 dBsm
    // 合理的有效范围：-100 ~ +100 dBsm（覆盖从极小目标到大型目标）
    if (std::isfinite(track_item->RCS) && track_item->RCS >= -100.0f && track_item->RCS <= 100.0f) {
        auto *info_rcs = detectionreport->add_object_info();
        info_rcs->set_type("RCS");
        char rcs_buf[32];
        snprintf(rcs_buf, sizeof(rcs_buf), "%.2fdBsm", track_item->RCS);
        info_rcs->set_value(rcs_buf);
    }
    
    // 11. 跟踪类型（TWS/TAS）
    auto *info_track_type = detectionreport->add_object_info();
    info_track_type->set_type("trackType");
    info_track_type->set_value(track_item->twsTasFlag == 0 ? "TWS" : "TAS");
    
    // 12. 航迹状态类型：有效范围 0 ~ 1 (0: 暂态航迹, 1: 稳态航迹)
    if (track_item->state_type <= 1) {
        auto *info_state_type = detectionreport->add_object_info();
        info_state_type->set_type("trackState");
        info_state_type->set_value(track_item->state_type == 1 ? "Confirmed" : "Tentative");
    }
    
    // 13. 航向角
    if (track_item->orientationAngle >= 0.0f && track_item->orientationAngle <= 360.0f) {
        auto *info_heading = detectionreport->add_object_info();
        info_heading->set_type("heading");
        char heading_buf[32];
        snprintf(heading_buf, sizeof(heading_buf), "%.2f°", track_item->orientationAngle);
        info_heading->set_value(heading_buf);
    }
    
    // 14. 目标跟踪时长（体现跟踪稳定性）：有效范围 0 ~ 10000s
    if (track_item->alive >= 0.0f && track_item->alive <= 10000.0f) {
        auto *info_alive = detectionreport->add_object_info();
        info_alive->set_type("trackDuration");
        char alive_buf[32];
        snprintf(alive_buf, sizeof(alive_buf), "%.1fs", track_item->alive);
        info_alive->set_value(alive_buf);
    }
    
    // 15. 威胁评估 - 到达关注点最短时间（TOCA）：有效范围 0 ~ 1000000ms
    // if (track_item->TOCA >= 0.0f && track_item->TOCA <= 1000000.0f) {
    //     auto *info_toca = detectionreport->add_object_info();
    //     info_toca->set_type("threatTOCA");
    //     char toca_buf[32];
    //     snprintf(toca_buf, sizeof(toca_buf), "%.1fs", track_item->TOCA / 1000.0f);  // ms转s
    //     info_toca->set_value(toca_buf);
    // }
    
    // 16. 威胁评估 - 距离关注点最近距离（DOCA）：有效范围 0 ~ 6000m
    // if (track_item->DOCA >= 0.0f && track_item->DOCA <= 6000.0f) {
    //     auto *info_doca = detectionreport->add_object_info();
    //     info_doca->set_type("threatDOCA");
    //     char doca_buf[32];
    //     snprintf(doca_buf, sizeof(doca_buf), "%.2fm", track_item->DOCA);
    //     info_doca->set_value(doca_buf);
    // }

    // 目标分类（按照 SAPIENT 官方分类标准 - BSI Flex 335 v2.0 Table 96）
    auto *classification = detectionreport->add_classification();
    
    // 类别置信度（0-1）
    float class_confidence = track_item->classifyProb / 100.0f;
    if (class_confidence > 1.0f) class_confidence = 1.0f;
    if (class_confidence < 0.0f) class_confidence = 0.0f;
    
    // 根据雷达分类映射到 SAPIENT 标准分类
    // 雷达分类定义（来自 radar.pb.h）：
    // 0x00：未识别  0x01：无人机  0x02：单兵  0x03：车辆  0x04：鸟类  0x05：直升机
    switch (track_item->classification) {
        case 0x00: // 未识别
            classification->set_type("Unknown");
            classification->set_confidence(class_confidence);
            break;
            
        case 0x01: // 无人机 -> Air vehicle > UAV rotary wing
            classification->set_type("Air vehicle");
            classification->set_confidence(class_confidence);
            {
                auto *subclass = classification->add_sub_class();
                subclass->set_type("UAV rotary wing");
                subclass->set_level(1);
                subclass->set_confidence(class_confidence);
            }
            break;
            
        case 0x02: // 单兵 -> Human
            classification->set_type("Human");
            classification->set_confidence(class_confidence);
            break;
            
        case 0x03: // 车辆 -> Land vehicle
            classification->set_type("Land vehicle");
            classification->set_confidence(class_confidence);
            break;
            
        case 0x04: // 鸟类 -> Animal > Bird
            classification->set_type("Animal");
            classification->set_confidence(class_confidence);
            {
                auto *subclass = classification->add_sub_class();
                subclass->set_type("Bird");
                subclass->set_level(1);
                subclass->set_confidence(class_confidence);
            }
            break;
            
        default: // 其他未知情况 -> Other
            classification->set_type("Other");
            classification->set_confidence(class_confidence);
            break;
    }

    // 行为：根据运动类型设置
    // 运动类型定义（来自 radar.pb.h）：
    // 0：未知  1：静止  2：悬停  3：靠近  4：远离
    // 注意：雷达协议中 motionType 没有对应的置信度字段，因此 behaviour 不设置 confidence
    auto *behaviour = detectionreport->add_behaviour();

    // motionType 在部分“假数据/未初始化数据”场景可能一直为 0（未知），
    // 这里增加兜底：用速度信息推断 Active/Passive，避免对端全部显示 other。
    switch (track_item->motionType) {
        case 1: // 静止 -> Passive
            behaviour->set_type("Passive");
            break;

        case 2: // 悬停
        case 3: // 靠近
        case 4: // 远离
            behaviour->set_type("Active");
            break;

        default: {
            // 兜底：优先用 absVel，其次用径向速度/ENU 速度分量判断是否“有运动”
            const float abs_speed = fabsf(track_item->absVel);
            const float radial_speed = fabsf(track_item->velocity);
            const float enu_speed_hint = fabsf(track_item->vx) + fabsf(track_item->vy) + fabsf(track_item->vz);

            // 经验阈值：>0.5m/s 视为“有运动”
            const float ACTIVE_SPEED_THRESHOLD = 0.5f;
            if (abs_speed > ACTIVE_SPEED_THRESHOLD ||
                radial_speed > ACTIVE_SPEED_THRESHOLD ||
                enu_speed_hint > ACTIVE_SPEED_THRESHOLD) {
                behaviour->set_type("Active");
            } else {
                // 无法判断/几乎静止：用 Passive 比 Other 更有信息量
                behaviour->set_type("Passive");
            }
            break;
        }
    }

    // 速度：使用 ENU 速度（东-北-天）
    if (track_item->vx != 0.0f || track_item->vy != 0.0f || track_item->vz != 0.0f) {
        auto *velocity = detectionreport->mutable_enu_velocity();
        
        // RadarTrackItem 中的速度坐标系为"北西天"（NWU）：
        //   vx: 北向速度（North）
        //   vy: 西向速度（West）
        //   vz: 天向速度（Up）
        // 需要转换为 SAPIENT 的"东北天"（ENU）坐标系：
        //   east_rate: 东向速度（East）
        //   north_rate: 北向速度（North）
        //   up_rate: 天向速度（Up）
        // 转换关系：
        //   East = -West （东向 = -西向）
        //   North = North （北向保持不变）
        //   Up = Up （天向保持不变）
        
        double east_rate = -track_item->vy;   // 东向 = -西向
        double north_rate = track_item->vx;   // 北向（不变）
        double up_rate = track_item->vz;      // 天向（不变）
        
        // 速度值范围检查：根据 RadarTrackItem 规范，速度范围为 -100~100 m/s
        const double MIN_VELOCITY = -100.0;
        const double MAX_VELOCITY = 100.0;
        
        // 限制速度值在有效范围内（先进行范围检查）
        if (east_rate < MIN_VELOCITY) {
            east_rate = MIN_VELOCITY;
        } else if (east_rate > MAX_VELOCITY) {
            east_rate = MAX_VELOCITY;
        }
        
        if (north_rate < MIN_VELOCITY) {
            north_rate = MIN_VELOCITY;
        } else if (north_rate > MAX_VELOCITY) {
            north_rate = MAX_VELOCITY;
        }
        
        if (up_rate < MIN_VELOCITY) {
            up_rate = MIN_VELOCITY;
        } else if (up_rate > MAX_VELOCITY) {
            up_rate = MAX_VELOCITY;
        }
        
        // 如果速度接近 0，使用最小值避免字段为空（在范围检查之后处理）
        const double MIN_SPEED = 0.001;
        if (fabs(east_rate) < 0.0001) east_rate = MIN_SPEED;
        if (fabs(north_rate) < 0.0001) north_rate = MIN_SPEED;
        
        velocity->set_east_rate(east_rate);
        velocity->set_north_rate(north_rate);
        velocity->set_up_rate(up_rate);
        
        // 误差根据速度方差估算（vx 和 vy 方差应该相同，用于东向和北向）
        double v_error = sqrt(track_item->vx_variance);
        if (v_error < 0.5) v_error = 0.5;
        velocity->set_east_rate_error(v_error);
        velocity->set_north_rate_error(v_error);
        velocity->set_up_rate_error(v_error);
    }

    // ID：使用 track ID
    char track_id_str[32];
    snprintf(track_id_str, sizeof(track_id_str), "track_%u", track_item->id);
    detectionreport->set_id(track_id_str);

    // 构造 SapientMessage wrapper
    sapient_msg::bsi_flex_335_v2_0::SapientMessage wrapper;
    if (detectrepo.nodeid().size() > 0) {
        wrapper.set_node_id(detectrepo.nodeid());
    }
    
    // 顶层 timestamp
    {
        auto now = std::chrono::system_clock::now();
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        auto nanos_total = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        int nanos = static_cast<int>(nanos_total - secs * 1000000000LL);
        google::protobuf::Timestamp* ts = wrapper.mutable_timestamp();
        ts->set_seconds(static_cast<long long>(secs));
        ts->set_nanos(nanos);
    }
    
    wrapper.mutable_detection_report()->CopyFrom(detectrepo.detectionreport());

    // 序列化
    if (!wrapper.SerializeToString(&out_serialized)) {
        std::cerr << "序列化 SapientMessage wrapper 失败" << std::endl;
        return -1;
    }

    // 序列化为 JSON
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
    // 为 sapient_tcp.cpp 暴露的 C++ 接口
    // 基于 RadarTrackItem（应用层数据，0x12 消息）
    int sapient_build_detection_report_from_track_item_cpp(std::string &out_serialized, std::string &out_json, const RadarTrackItem *track_item)
    {
        return sapient_build_detection_report_from_track_item(out_serialized, out_json, track_item);
    }

    // 基于 RadarTrackItem（应用层数据，0x12 消息）发送 DetectionReport
    int sapient_detect_from_track_item(sapient_tcp_client_t *client, const RadarTrackItem *track_item)
    {
        if (!client || !track_item) {
            return -1;
        }
        return sapient_tcp_client_send_detection_report_from_track_item(client, track_item);
    }
}