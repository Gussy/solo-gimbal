#ifndef GOPRO_MAV_CONVERTERS_H
#define GOPRO_MAV_CONVERTERS_H

#include "mavlink_interface/gimbal_mavlink.h"
#include "gopro_hero3p_defs.h"
#include "gopro_hero4_defs.h"

#include <stdbool.h>

// hero 3

GOPRO_CAPTURE_MODE h3p_to_mav_cap_mode(uint8_t mode, bool *ok);
H3P_CAPTURE_MODE mav_to_h3p_cap_mode(uint8_t mode, bool *ok);

GOPRO_RESOLUTION h3p_to_mav_res(uint8_t res, bool *ok);
H3P_RESOLUTION mav_to_h3p_res(uint8_t res, bool *ok);

GOPRO_FRAME_RATE h3p_to_mav_framerate(uint8_t rate, bool *ok);
H3P_FRAMERATE mav_to_h3p_framerate(uint8_t rate, bool *ok);

GOPRO_PHOTO_RESOLUTION h3p_to_mav_photo_res(uint8_t res, bool *ok);
H3P_PHOTO_RESOLUTION mav_to_h3p_photo_res(uint8_t res, bool *ok);

GOPRO_PROTUNE_EXPOSURE h3p_to_mav_exposure(uint8_t exp, bool *ok);
H3P_PROTUNE_EXPOSURE mav_to_h3p_exposure(uint8_t exp, bool *ok);

// hero 4

GOPRO_CAPTURE_MODE h4_to_mav_cap_mode(uint8_t mode, bool *ok);
GP_H4_CAPTURE_MODE mav_to_h4_cap_mode(uint8_t mode, bool *ok);

GOPRO_RESOLUTION h4_to_mav_resolution(uint8_t res, bool *ok);
H4_RESOLUTION mav_to_h4_resolution(uint8_t rate, bool *ok);

GOPRO_FRAME_RATE h4_to_mav_framerate(uint8_t rate, bool *ok);
H4_FRAMERATE mav_to_h4_framerate(uint8_t rate, bool *ok);

GOPRO_PHOTO_RESOLUTION h4_to_mav_photo_res(uint8_t res, bool *ok);
H4_PHOTO_RESOLUTION mav_to_h4_photo_res(uint8_t res, bool *ok);

GOPRO_PROTUNE_EXPOSURE h4_to_mav_exposure(uint8_t exp, bool *ok);
H4_PROTUNE_EXPOSURE mav_to_h4_exposure(uint8_t exp, bool *ok);

#endif // GOPRO_MAV_CONVERTERS_H
