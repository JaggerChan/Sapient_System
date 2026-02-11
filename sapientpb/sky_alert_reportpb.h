/*****************************************************************************
 * Alert builder (minimal version)
 * 构建最小 SAPIENT Alert -> SapientMessage 封装
 * 仅包含: alert_id (ULID), alert_type, status, description
 * 其余字段留待扩展。
 *****************************************************************************/
#ifndef __SKY_ALERT_REPORTPB_H_
#define __SKY_ALERT_REPORTPB_H_

#include <string>

/* 构建并封装 Alert 消息到 SapientMessage
 * 参数:
 *  description      可选的描述字符串 (NULL 则使用默认 "system alert")
 *  type             Alert 类型枚举值(参考 Alert::AlertType)，0 或非法则使用 INFORMATION
 *  status           Alert 状态枚举值(参考 Alert::AlertStatus)，0 或非法则使用 ACTIVE
 * 输出:
 *  out_serialized   SapientMessage 二进制 (带 timestamp/node_id/alert)
 *  out_json         便于调试的 JSON 文本
 * 返回: 0 成功, 负值失败
 */
int sapient_build_alert_report(std::string &out_serialized,
                               std::string &out_json,
                               const char *description,
                               int type,
                               int status);

#endif /* __SKY_ALERT_REPORTPB_H_ */
