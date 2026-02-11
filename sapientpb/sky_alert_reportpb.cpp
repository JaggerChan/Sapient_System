#include "sky_alert_reportpb.h"
#include "../sapient/alert.pb.h"
#include "../sapient/sapient_message.pb.h"
#include "sapient_nodeid.h"
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/timestamp.pb.h>
#include <chrono>
#include <string>
#include <iostream>

// extern std::string g_nodeId; // Replaced by sapient_nodeid.h
extern "C" void generate_ulid(char *ulid); // 复用已有 ULID 生成函数

// 设置当前 UTC 时间戳 (与其它 builder 保持一致)
static void set_current_timestamp(::google::protobuf::Timestamp *ts)
{
    auto now = std::chrono::system_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
    ts->set_seconds(seconds);
    ts->set_nanos((int32_t)nanos);
}

int sapient_build_alert_report(std::string &out_serialized,
                               std::string &out_json,
                               const char *description,
                               int type,
                               int status)
{
    using namespace sapient_msg::bsi_flex_335_v2_0;

    Alert alert;

    // 必需: alert_id ULID
    char ulid[27] = {0};
    generate_ulid(ulid);
    alert.set_alert_id(ulid);

    // alert_type (默认 INFORMATION)
    Alert::AlertType atype = Alert::ALERT_TYPE_INFORMATION;
    if (type >= Alert::ALERT_TYPE_INFORMATION && type <= Alert::ALERT_TYPE_MODE_CHANGE) {
        atype = static_cast<Alert::AlertType>(type);
    }
    alert.set_alert_type(atype);

    // status (默认 ACTIVE)
    Alert::AlertStatus astatus = Alert::ALERT_STATUS_ACTIVE;
    if (status >= Alert::ALERT_STATUS_ACTIVE && status <= Alert::ALERT_STATUS_CLEAR) {
        astatus = static_cast<Alert::AlertStatus>(status);
    }
    alert.set_status(astatus);

    // description (默认字符串)
    if (description && *description) {
        alert.set_description(description);
    } else {
        alert.set_description("system alert");
    }

    // 封装到 SapientMessage
    SapientMessage wrapper;
    set_current_timestamp(wrapper.mutable_timestamp());
    std::string node_id = generateNodeID();
    if (!node_id.empty()) {
        wrapper.set_node_id(node_id);
    }
    wrapper.set_allocated_alert(new Alert(alert));

    if (!wrapper.SerializeToString(&out_serialized)) {
        std::cerr << "Failed to serialize Alert wrapper" << std::endl;
        return -1;
    }

    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    options.always_print_primitive_fields = true;
    if (!google::protobuf::util::MessageToJsonString(wrapper, &out_json, options).ok()) {
        std::cerr << "Failed to convert Alert wrapper to JSON" << std::endl;
        return -2;
    }

    return 0;
}
