#ifndef GOPRO_CAN_MAVLINK_H
#define GOPRO_CAN_MAVLINK_H

#include "f2806x_int8.h"

/*
 * gp_can_mavlink_* types correpond to gopro mavlink types,
 * but omit additional information not required to be sent
 * internally within the gimbal over the CAN interface,
 * most often the target system/component.
 *
 * At the moment, these must all be sized to fit in a single
 * CAN frame.
 *
 * Unfortunately, the c2000 toolchain appears to be unable
 * to cope with anonymous structs within unions, so we
 * name them and deal with the extra typing.
 */

typedef union {
    struct {
        uint8_t hb_status;
        uint8_t capture_mode;
        uint8_t flags;      // see GOPRO_HEARTBEAT_FLAGS
    } mav;
    uint8_t bytes[3];
} gp_can_mav_heartbeat_t;   // corresponds to mavlink_gopro_heartbeat_t

typedef union {
    struct {
        uint8_t cmd_id;
    } mav;
    uint8_t bytes[1];
} gp_can_mav_get_req_t;     // corresponds to mavlink_gopro_get_request_t

typedef union {
    struct {
        uint8_t cmd_id;
        uint8_t value[4];
    } mav;
    uint8_t bytes[5];
} gp_can_mav_set_req_t;     // corresponds to mavlink_gopro_set_request_t

typedef union {
    struct {
        uint8_t cmd_id;
        uint8_t status;
        uint8_t value[4];
    } mav;
    uint8_t bytes[6];
} gp_can_mav_get_rsp_t;     // corresponds to mavlink_gopro_get_response_t

typedef union {
    struct {
        uint8_t cmd_id;
        uint8_t status;
    } mav;
    uint8_t bytes[2];
} gp_can_mav_set_rsp_t;     // corresponds to mavlink_gopro_set_response_t

#endif // GOPRO_CAN_MAVLINK_H
