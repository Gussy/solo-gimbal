#ifndef _GOPRO_HERO4_DEFS_H_
#define _GOPRO_HERO4_DEFS_H_

/*
 * Definitions only meant for consumption in gopro_hero4.c
 */

enum GP_H4_ACK_VALS {
    H4_ACK  = 0,
    H4_NACK = 1
};

enum GP_H4_TCB_VALS {
    TCB_CMD_SINGLE_PKT  = 0x0,
    TCB_RSP_FINAL_FRAME = 0x10
};

enum GP_ZZ_FMT {
    ZZ_I2C_BUS_SPEED    = 1,    // 0 == standard
    ZZ_RDY              = 2,    // 1 == ready
    ZZ_FW_VERSION       = 3,    // fmt: "HD4.0x.MM.mm.rr"
    ZZ_PROTO_VERSION    = 4     // fmt: MM, mm, rr (3 bytes)
};

enum GP_H4_HANDSHAKE_STEPS {
    GP_H4_HANDSHAKE_NONE,
    GP_H4_HANDSHAKE_READY,              // have received `ZZ ready` cmd from camera
    GP_H4_HANDSHAKE_HB_PROTO_VERSION,   // have receive `ZZ HeroBus Protocol Version` from camera
    GP_H4_HANDSHAKE_CHANNEL_OPEN        // have opened the comms channel with the camera
};

typedef enum {
    H4_CAPTURE_MODE_VIDEO       = 0,
    H4_CAPTURE_MODE_PHOTO       = 1,
    H4_CAPTURE_MODE_MULTI_SHOT  = 2,
    H4_CAPTURE_MODE_PLAYBACK    = 4,
    H4_CAPTURE_MODE_SETUP       = 5,
    H4_CAPTURE_MODE_UNKNOWN     = 0xee  // arbitrary invalid value
} GP_H4_CAPTURE_MODE;

enum GP_H4_API_GROUP {
    API_GRP_GEN_CMDS        = 0,
    API_GRP_MODE_CAM        = 1,
    API_GRP_MODE_VID        = 2,
    API_GRP_MODE_PHOTO      = 3,
    API_GRP_MODE_MULTISHOT  = 4,
    API_GRP_PLAYBACK_MODE   = 7,
    API_GRP_CAM_SETTINGS    = 8,
    API_GRP_FS_SETTINGS     = 9
};

enum GP_H4_API_ID {
    API_ID_OPEN_CHAN                = 0x1,
    API_ID_CLOSE_CHAN               = 0x2,
    API_ID_SET_CAM_MAIN_MODE        = 0x1,
    API_ID_GET_CAM_MAIN_MODE        = 0x0,
    API_ID_SET_SUBMODE              = 0x3,    // depends on current main mode
    API_ID_GET_SUBMODE              = 0x2,
    API_ID_TRIGGER_VID_START        = 0x1b,
    API_ID_TRIGGER_VID_STOP         = 0x1c,
    API_ID_TRIGGER_PHOTO_START      = 0x17,
    API_ID_TRIGGER_PHOTO_STOP       = 0x18,
    API_ID_TRIGGER_MULTI_START      = 0x1b,
    API_ID_TRIGGER_MULTI_STOP       = 0x1c,
    API_ID_SET_RES_FR_FOV           = 0x3,    // sets resolution, frame rate, and field of view
    API_ID_GET_RES_FR_FOV           = 0x2,
    API_ID_SET_VID_PROTUNE          = 0xf,
    API_ID_GET_VID_PROTUNE          = 0xe,
    API_ID_SET_VID_EXPOSURE         = 0x19,
    API_ID_GET_VID_EXPOSURE         = 0x18,
    API_ID_SET_NTSC_PAL             = 0x13,
    API_ID_GET_NTSC_PAL             = 0x12,
    API_ID_SET_VID_LOOPING          = 0x7,
    API_ID_GET_VID_LOOPING          = 0x6,
    API_ID_SET_VID_LOW_LIGHT        = 0x9,
    API_ID_GET_VID_LOW_LIGHT        = 0x8,
    API_ID_SET_PHOTO_RES            = 0x3,
    API_ID_GET_PHOTO_RES            = 0x2,
    API_ID_SET_PHOTO_EXPOSURE       = 0x15,
    API_ID_GET_PHOTO_EXPOSURE       = 0x14,
    API_ID_SET_PHOTO_PROTUNE        = 0xb,
    API_ID_GET_PHOTO_PROTUNE        = 0xa,
    API_ID_SET_CONT_SHUT_SPEED      = 0x5,  // photo
    API_ID_GET_CONT_SHUT_SPEED      = 0x4,
    API_ID_SET_BURST_SHUT_RATE      = 0x5,  // multishot
    API_ID_GET_BURST_SHUT_RATE      = 0x4,
    API_ID_SET_TIME_LAPSE_INTERVAL  = 0x7,
    API_ID_GET_TIME_LAPSE_INTERVAL  = 0x6,
    API_ID_SET_MULTI_EXPOSURE       = 0x19,
    API_ID_GET_MULTI_EXPOSURE       = 0x18,
    API_ID_SET_MULTI_PROTUNE        = 0xf,
    API_ID_GET_MULTI_PROTUNE        = 0xe,
    API_ID_SET_OSD                  = 0x15,
    API_ID_GET_OSD                  = 0x14,
    API_ID_POWER_OFF                = 0x2,
    API_ID_GET_BATTERY_LVL          = 0x0,
    API_ID_FORMAT_SD_CARD           = 0xa,
    API_ID_DELETE_LAST_FILE         = 0x9,
    API_ID_SET_BLINK_RATE           = 0xf,
    API_ID_GET_BLINK_RATE           = 0xe,
    API_ID_SET_BEEP_VOLUME          = 0x11,
    API_ID_GET_BEEP_VOLUME          = 0x10,
    API_ID_SET_DEFAULT_MAIN_MODE    = 0xb,
    API_ID_GET_DEFAULT_MAIN_MODE    = 0xa,
    API_ID_SET_DEFAULT_SUB_MODE     = 0x1,  // api group specifies vid/photo/multi
    API_ID_GET_DEFAULT_SUB_MODE     = 0x2,
    API_ID_SET_QUICK_CAP_MODE       = 0xd,
    API_ID_GET_QUICK_CAP_MODE       = 0xc,
    API_ID_SET_AUTO_PWR_OFF_DELAY   = 0x17, // XXX: auto power off & camera time both have the same group/id combo in datasheet
    API_ID_GET_AUTO_PWR_OFF_DELAY   = 0x16, //      still need to figure out which is correct
    API_ID_SET_CAM_TIME             = 0x17,
    API_ID_GET_CAM_TIME             = 0x16,
    API_ID_SET_LOCATE_CAM_ON        = 0xd,
    API_ID_SET_LOCATE_CAM_OFF       = 0xc
};

#endif // _GOPRO_HERO4_DEFS_H_