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
    GP_H4_HANDSHARE_CHAN_OPEN_ERR,      // gopro responded to 'open chan' cmd with error, currently unrecoverable
    GP_H4_HANDSHAKE_CHANNEL_OPEN        // have opened the comms channel with the camera
};

typedef enum {
    H4_CAPTURE_MODE_VIDEO       = 0,
    H4_CAPTURE_MODE_PHOTO       = 1,
    H4_CAPTURE_MODE_MULTI_SHOT  = 2,
    H4_CAPTURE_MODE_PLAYBACK    = 4,
    H4_CAPTURE_MODE_SETUP       = 5
} GP_H4_CAPTURE_MODE;

typedef enum {
    H4_RESOLUTION_4k_17_9           = 0x0,
    H4_RESOLUTION_4k                = 0x1,
    H4_RESOLUTION_4k_SUPERVIEW      = 0x2,
    H4_RESOLUTION_2_7k_17_9         = 0x3,
    H4_RESOLUTION_2_7k_16_9         = 0x4,
    H4_RESOLUTION_2_7k_SUPERVIEW    = 0x5,
    H4_RESOLUTION_2_7k_4_3          = 0x6,
    H4_RESOLUTION_1440p             = 0x7,
    H4_RESOLUTION_1080p_SUPERVIEW   = 0x8,
    H4_RESOLUTION_1080p             = 0x9,
    H4_RESOLUTION_960p              = 0xa,
    H4_RESOLUTION_720p_SUPERVIEW    = 0xb,
    H4_RESOLUTION_720p              = 0xc,
    H4_RESOLUTION_480p              = 0xd,
} H4_RESOLUTION;

typedef enum {
    H4_FRAMERATE_240fps     = 0x0,
    H4_FRAMERATE_120fps     = 0x1,
    H4_FRAMERATE_100fps     = 0x2,
    H4_FRAMERATE_90fps      = 0x3,
    H4_FRAMERATE_80fps      = 0x4,
    H4_FRAMERATE_60fps      = 0x5,
    H4_FRAMERATE_50fps      = 0x6,
    H4_FRAMERATE_48fps      = 0x7,
    H4_FRAMERATE_30fps      = 0x8,
    H4_FRAMERATE_25fps      = 0x9,
    H4_FRAMERATE_24fps      = 0xa,
    H4_FRAMERATE_15fps      = 0xb,
    H4_FRAMERATE_12_5fps    = 0xc
} H4_FRAMERATE;

typedef enum {
    H4_PHOTO_RES_12MP_WIDE      = 0x0,
    H4_PHOTO_RES_7MP_WIDE       = 0x1,
    H4_PHOTO_RES_7MP_MEDIUM     = 0x2,
    H4_PHOTO_RES_5MP_MEDIUM     = 0x3
} H4_PHOTO_RESOLUTION;

typedef enum {
    H4_PROTUNE_EXPOSURE_POS_2_0     = 0x0,
    H4_PROTUNE_EXPOSURE_POS_1_5     = 0x1,
    H4_PROTUNE_EXPOSURE_POS_1_0     = 0x2,
    H4_PROTUNE_EXPOSURE_POS_0_5     = 0x3,
    H4_PROTUNE_EXPOSURE_ZERO        = 0x4,
    H4_PROTUNE_EXPOSURE_NEG_0_5     = 0x5,
    H4_PROTUNE_EXPOSURE_NEG_1_0     = 0x6,
    H4_PROTUNE_EXPOSURE_NEG_1_5     = 0x7,
    H4_PROTUNE_EXPOSURE_NEG_2_0     = 0x8
} H4_PROTUNE_EXPOSURE;

enum H4_TV_MODE {
    H4_TV_NTSC      = 0x0,
    H4_TV_PAL       = 0x1
};

typedef enum {
    H4_BURST_RATE_3_IN_1_SECOND     = 0x0,
    H4_BURST_RATE_5_IN_1_SECOND     = 0x1,
    H4_BURST_RATE_10_IN_1_SECOND    = 0x2,
    H4_BURST_RATE_10_IN_2_SECOND    = 0x3,
    H4_BURST_RATE_10_IN_3_SECOND    = 0x4,
    H4_BURST_RATE_30_IN_1_SECOND    = 0x5,
    H4_BURST_RATE_30_IN_2_SECOND    = 0x6,
    H4_BURST_RATE_30_IN_3_SECOND    = 0x7,
    H4_BURST_RATE_30_IN_6_SECOND    = 0x8
} H4_BURST_RATE;

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
    API_ID_SET_AUTO_PWR_OFF_DELAY   = 0x17,
    API_ID_GET_AUTO_PWR_OFF_DELAY   = 0x16,
    API_ID_SET_CAM_TIME             = 0x1b, // datasheet has a typo for group/id for set/get time
    API_ID_GET_CAM_TIME             = 0x1a,
    API_ID_SET_LOCATE_CAM_ON        = 0xd,
    API_ID_SET_LOCATE_CAM_OFF       = 0xc
};

#endif // _GOPRO_HERO4_DEFS_H_
