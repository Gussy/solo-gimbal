#ifndef _GOPRO_HERO_COMMON
#define _GOPRO_HERO_COMMON

// TODO: determine what actually should exist here, may have been overzealous
#include "PeripheralHeaderIncludes.h"

typedef enum {
    GP_CMD_STATUS_SUCCESS = 0,
    GP_CMD_STATUS_FAILURE = 1,
    GP_CMD_STATUS_INCOMPLETE = 2    // either not started, or in progress
} GPCmdStatus;

typedef enum {
    GP_REQUEST_NONE,
    GP_REQUEST_GET,
    GP_REQUEST_SET
} GPRequestType;

// TODO: the use of this struct might change based on pending changes to how information is passed through MAVLink
typedef enum {
    GP_RECORDING_STOP,
    GP_RECORDING_START,
} GPRecording;

#endif
