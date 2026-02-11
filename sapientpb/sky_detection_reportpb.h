#ifndef _DETECTION_REPORT_PB_H_
#define _DETECTION_REPORT_PB_H_

#include "sapient_tcp.h"

#ifdef __cplusplus
extern "C" {
#endif

// 基于 RadarTrackItem（应用层数据，0x12 消息）构建并发送 DetectionReport
// client: TCP 客户端句柄
// track_item: 目标跟踪项（来自 RadarTrack，应用层数据）
// 返回 0 表示成功，负值表示失败
int sapient_detect_from_track_item(sapient_tcp_client_t *client, const RadarTrackItem *track_item);

#ifdef __cplusplus
}
#endif



#endif