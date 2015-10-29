#ifndef _GOPRO_HERO_COMMON
#define _GOPRO_HERO_COMMON

// TODO: determine what actually should exist here, may have been overzealous
#include "PeripheralHeaderIncludes.h"

typedef enum {
    GP_CMD_STATUS_SUCCESS       = 0,
    GP_CMD_STATUS_FAILURE       = 1,
    GP_CMD_STATUS_IDLE          = 2,
    GP_CMD_STATUS_IN_PROGRESS   = 3
} GPCmdStatus;

typedef enum {
    GP_REQUEST_NONE,
    GP_REQUEST_GET,
    GP_REQUEST_SET
} GPRequestType;

#endif
