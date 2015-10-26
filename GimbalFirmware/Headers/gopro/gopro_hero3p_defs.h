#ifndef GOPRO_HERO3P_DEFS_H
#define GOPRO_HERO3P_DEFS_H

/*
 * Definitions meant for consumption in gopro_hero3p.c
 */

typedef enum {
    H3P_CAPTURE_MODE_VIDEO      = 0,
    H3P_CAPTURE_MODE_PHOTO      = 1,
    H3P_CAPTURE_MODE_BURST      = 2,
    H3P_CAPTURE_MODE_TIME_LAPSE = 3
} H3P_CAPTURE_MODE;

typedef enum {
    H3P_RES_480p            = 0x0,
    H3P_RES_720p            = 0x1,
    H3P_RES_960p            = 0x2,
    H3P_RES_1080p           = 0x3,
    H3P_RES_1440p           = 0x4,
    H3P_RES_2_7k_16_9       = 0x5,
    H3P_RES_4k_16_9         = 0x6,
    H3P_RES_2_7k_17_9       = 0x7,
    H3P_RES_4k_17_9         = 0x8,
    H3P_RES_1080p_SUPER     = 0x9,
    H3P_RES_720p_SUPER      = 0xa
} H3P_RESOLUTION;

typedef enum {
    H3P_FRAMERATE_12fps     = 0x0,
    H3P_FRAMERATE_15fps     = 0x1,
    H3P_FRAMERATE_24fps     = 0x2,
    H3P_FRAMERATE_25fps     = 0x3,
    H3P_FRAMERATE_30fps     = 0x4,
    H3P_FRAMERATE_48fps     = 0x5,
    H3P_FRAMERATE_50fps     = 0x6,
    H3P_FRAMERATE_60fps     = 0x7,
    H3P_FRAMERATE_100fps    = 0x8,
    H3P_FRAMERATE_120fps    = 0x9,
    H3P_FRAMERATE_240fps    = 0xa,
    H3P_FRAMERATE_12_5fps   = 0xb,
} H3P_FRAMERATE;

// for now, mavlink defs for Field of View match hero bus, no need to convert

#endif // GOPRO_HERO3P_DEFS_H
