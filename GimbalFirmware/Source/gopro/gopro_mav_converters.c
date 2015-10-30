#include "gopro_mav_converters.h"

/*
 * Conversion routines between herobus and mavlink values.
 *                      This is tedium.
 */

// helpers to reduce at least a little boilerplate
#define default_true_open_switch(v) *ok = true; switch ((v))
#define close_switch_return_default(t) default: *ok = false; return (t)0

////////////////////////////
// hero 3 capture mode
////////////////////////////

GOPRO_CAPTURE_MODE h3p_to_mav_cap_mode(uint8_t mode, bool *ok)
{
    default_true_open_switch(mode) {
    case H3P_CAPTURE_MODE_VIDEO:        return GOPRO_CAPTURE_MODE_VIDEO;
    case H3P_CAPTURE_MODE_PHOTO:        return GOPRO_CAPTURE_MODE_PHOTO;
    case H3P_CAPTURE_MODE_BURST:        return GOPRO_CAPTURE_MODE_BURST;
    case H3P_CAPTURE_MODE_TIME_LAPSE:   return GOPRO_CAPTURE_MODE_TIME_LAPSE;
    close_switch_return_default(GOPRO_CAPTURE_MODE);
    }
}

H3P_CAPTURE_MODE mav_to_h3p_cap_mode(uint8_t mode, bool *ok)
{
    default_true_open_switch(mode) {
    case GOPRO_CAPTURE_MODE_VIDEO:      return H3P_CAPTURE_MODE_VIDEO;
    case GOPRO_CAPTURE_MODE_PHOTO:      return H3P_CAPTURE_MODE_PHOTO;
    case GOPRO_CAPTURE_MODE_BURST:      return H3P_CAPTURE_MODE_BURST;
    case GOPRO_CAPTURE_MODE_TIME_LAPSE: return H3P_CAPTURE_MODE_TIME_LAPSE;
    close_switch_return_default(H3P_CAPTURE_MODE);
    }
}

////////////////////////////
// hero 3 resolution
////////////////////////////

GOPRO_RESOLUTION h3p_to_mav_res(uint8_t res, bool *ok)
{
    default_true_open_switch(res) {
    case H3P_RES_480p:          return GOPRO_RESOLUTION_480p;
    case H3P_RES_720p:          return GOPRO_RESOLUTION_720p;
    case H3P_RES_960p:          return GOPRO_RESOLUTION_960p;
    case H3P_RES_1080p:         return GOPRO_RESOLUTION_1080p;
    case H3P_RES_1440p:         return GOPRO_RESOLUTION_1440p;
    case H3P_RES_2_7k_16_9:     return GOPRO_RESOLUTION_2_7k_16_9;
    case H3P_RES_4k_16_9:       return GOPRO_RESOLUTION_4k_16_9;
    case H3P_RES_2_7k_17_9:     return GOPRO_RESOLUTION_2_7k_17_9;
    case H3P_RES_4k_17_9:       return GOPRO_RESOLUTION_4k_17_9;
    case H3P_RES_1080p_SUPER:   return GOPRO_RESOLUTION_1080p_SUPERVIEW;
    case H3P_RES_720p_SUPER:    return GOPRO_RESOLUTION_720p_SUPERVIEW;
    close_switch_return_default(GOPRO_RESOLUTION);
    }
}

H3P_RESOLUTION mav_to_h3p_res(uint8_t res, bool *ok)
{
    default_true_open_switch(res) {
    case GOPRO_RESOLUTION_480p:             return H3P_RES_480p;
    case GOPRO_RESOLUTION_720p:             return H3P_RES_720p;
    case GOPRO_RESOLUTION_960p:             return H3P_RES_960p;
    case GOPRO_RESOLUTION_1080p:            return H3P_RES_1080p;
    case GOPRO_RESOLUTION_1440p:            return H3P_RES_1440p;
    case GOPRO_RESOLUTION_2_7k_16_9:        return H3P_RES_2_7k_16_9;
    case GOPRO_RESOLUTION_4k_16_9:          return H3P_RES_4k_16_9;
    case GOPRO_RESOLUTION_2_7k_17_9:        return H3P_RES_2_7k_17_9;
    case GOPRO_RESOLUTION_4k_17_9:          return H3P_RES_4k_17_9;
    case GOPRO_RESOLUTION_1080p_SUPERVIEW:  return H3P_RES_1080p_SUPER;
    case GOPRO_RESOLUTION_720p_SUPERVIEW:   return H3P_RES_720p_SUPER;
    close_switch_return_default(H3P_RESOLUTION);
    }
}

////////////////////////////
// hero 3 framerate
////////////////////////////

GOPRO_FRAME_RATE h3p_to_mav_framerate(uint8_t rate, bool *ok)
{
    default_true_open_switch(rate) {
    case H3P_FRAMERATE_12fps:   return GOPRO_FRAME_RATE_12;
    case H3P_FRAMERATE_15fps:   return GOPRO_FRAME_RATE_15;
    case H3P_FRAMERATE_24fps:   return GOPRO_FRAME_RATE_24;
    case H3P_FRAMERATE_25fps:   return GOPRO_FRAME_RATE_25;
    case H3P_FRAMERATE_30fps:   return GOPRO_FRAME_RATE_30;
    case H3P_FRAMERATE_48fps:   return GOPRO_FRAME_RATE_48;
    case H3P_FRAMERATE_50fps:   return GOPRO_FRAME_RATE_50;
    case H3P_FRAMERATE_60fps:   return GOPRO_FRAME_RATE_60;
    case H3P_FRAMERATE_100fps:  return GOPRO_FRAME_RATE_100;
    case H3P_FRAMERATE_120fps:  return GOPRO_FRAME_RATE_120;
    case H3P_FRAMERATE_240fps:  return GOPRO_FRAME_RATE_240;
    case H3P_FRAMERATE_12_5fps: return GOPRO_FRAME_RATE_12_5;
    close_switch_return_default(GOPRO_FRAME_RATE);
    }
}

H3P_FRAMERATE mav_to_h3p_framerate(uint8_t rate, bool *ok)
{
    default_true_open_switch(rate) {
    case GOPRO_FRAME_RATE_12:       return H3P_FRAMERATE_12fps;
    case GOPRO_FRAME_RATE_15:       return H3P_FRAMERATE_15fps;
    case GOPRO_FRAME_RATE_24:       return H3P_FRAMERATE_24fps;
    case GOPRO_FRAME_RATE_25:       return H3P_FRAMERATE_25fps;
    case GOPRO_FRAME_RATE_30:       return H3P_FRAMERATE_30fps;
    case GOPRO_FRAME_RATE_48:       return H3P_FRAMERATE_48fps;
    case GOPRO_FRAME_RATE_50:       return H3P_FRAMERATE_50fps;
    case GOPRO_FRAME_RATE_60:       return H3P_FRAMERATE_60fps;
    case GOPRO_FRAME_RATE_100:      return H3P_FRAMERATE_100fps;
    case GOPRO_FRAME_RATE_120:      return H3P_FRAMERATE_120fps;
    case GOPRO_FRAME_RATE_240:      return H3P_FRAMERATE_240fps;
    case GOPRO_FRAME_RATE_12_5:     return H3P_FRAMERATE_12_5fps;
    close_switch_return_default(H3P_FRAMERATE);
    }
}

////////////////////////////
// hero 3 photo resolution
////////////////////////////

GOPRO_PHOTO_RESOLUTION h3p_to_mav_photo_res(uint8_t res, bool *ok)
{
    default_true_open_switch(res) {
    case H3P_PHOTO_RES_5MP_MEDIUM:  return GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM;
    case H3P_PHOTO_RES_7MP_WIDE:    return GOPRO_PHOTO_RESOLUTION_7MP_WIDE;
    case H3P_PHOTO_RES_12MP_WIDE:   return GOPRO_PHOTO_RESOLUTION_12MP_WIDE;
    case H3P_PHOTO_RES_7MP_MEDIUM:  return GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM;
    case H3P_PHOTO_RES_10MP_WIDE:   return GOPRO_PHOTO_RESOLUTION_10MP_WIDE;
    close_switch_return_default(GOPRO_PHOTO_RESOLUTION);
    }
}

H3P_PHOTO_RESOLUTION mav_to_h3p_photo_res(uint8_t res, bool *ok)
{
    default_true_open_switch(res) {
    case GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM:     return H3P_PHOTO_RES_5MP_MEDIUM;
    case GOPRO_PHOTO_RESOLUTION_7MP_WIDE:       return H3P_PHOTO_RES_7MP_WIDE;
    case GOPRO_PHOTO_RESOLUTION_12MP_WIDE:      return H3P_PHOTO_RES_12MP_WIDE;
    case GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM:     return H3P_PHOTO_RES_7MP_MEDIUM;
    case GOPRO_PHOTO_RESOLUTION_10MP_WIDE:      return H3P_PHOTO_RES_10MP_WIDE;
    close_switch_return_default(H3P_PHOTO_RESOLUTION);
    }
}

////////////////////////////
// hero 4 capture mode
////////////////////////////

GOPRO_CAPTURE_MODE h4_to_mav_cap_mode(uint8_t mode, bool *ok)
{
    default_true_open_switch(mode) {
    case H4_CAPTURE_MODE_VIDEO:         return GOPRO_CAPTURE_MODE_VIDEO;
    case H4_CAPTURE_MODE_PHOTO:         return GOPRO_CAPTURE_MODE_PHOTO;
    case H4_CAPTURE_MODE_MULTI_SHOT:    return GOPRO_CAPTURE_MODE_MULTI_SHOT;
    case H4_CAPTURE_MODE_PLAYBACK:      return GOPRO_CAPTURE_MODE_PLAYBACK;
    case H4_CAPTURE_MODE_SETUP:         return GOPRO_CAPTURE_MODE_SETUP;
    close_switch_return_default(GOPRO_CAPTURE_MODE);
    }
}

GP_H4_CAPTURE_MODE mav_to_h4_cap_mode(uint8_t mode, bool *ok)
{
    default_true_open_switch(mode) {
    case GOPRO_CAPTURE_MODE_VIDEO:      return H4_CAPTURE_MODE_VIDEO;
    case GOPRO_CAPTURE_MODE_PHOTO:      return H4_CAPTURE_MODE_PHOTO;
    case GOPRO_CAPTURE_MODE_MULTI_SHOT: return H4_CAPTURE_MODE_MULTI_SHOT;
    case GOPRO_CAPTURE_MODE_PLAYBACK:   return H4_CAPTURE_MODE_PLAYBACK;
    case GOPRO_CAPTURE_MODE_SETUP:      return H4_CAPTURE_MODE_SETUP;
    close_switch_return_default(GP_H4_CAPTURE_MODE);
    }
}

////////////////////////////
// hero 4 resolution
////////////////////////////

GOPRO_RESOLUTION h4_to_mav_resolution(uint8_t res, bool *ok)
{
    default_true_open_switch(res) {
    case H4_RESOLUTION_4k_17_9:         return GOPRO_RESOLUTION_4k_17_9;
    case H4_RESOLUTION_4k:              return GOPRO_RESOLUTION_4k_16_9;
    case H4_RESOLUTION_4k_SUPERVIEW:    return GOPRO_RESOLUTION_4k_SUPERVIEW;
    case H4_RESOLUTION_2_7k_17_9:       return GOPRO_RESOLUTION_2_7k_17_9;
    case H4_RESOLUTION_2_7k_16_9:       return GOPRO_RESOLUTION_2_7k_16_9;
    case H4_RESOLUTION_2_7k_SUPERVIEW:  return GOPRO_RESOLUTION_2_7k_SUPERVIEW;
    case H4_RESOLUTION_2_7k_4_3:        return GOPRO_RESOLUTION_2_7k_4_3;
    case H4_RESOLUTION_1440p:           return GOPRO_RESOLUTION_1440p;
    case H4_RESOLUTION_1080p_SUPERVIEW: return GOPRO_RESOLUTION_1080p_SUPERVIEW;
    case H4_RESOLUTION_1080p:           return GOPRO_RESOLUTION_1080p;
    case H4_RESOLUTION_960p:            return GOPRO_RESOLUTION_960p;
    case H4_RESOLUTION_720p_SUPERVIEW:  return GOPRO_RESOLUTION_720p_SUPERVIEW;
    case H4_RESOLUTION_720p:            return GOPRO_RESOLUTION_720p;
    case H4_RESOLUTION_480p:            return GOPRO_RESOLUTION_480p;
    close_switch_return_default(GOPRO_RESOLUTION);
    }
}

H4_RESOLUTION mav_to_h4_resolution(uint8_t rate, bool *ok)
{
    default_true_open_switch(rate) {
    case GOPRO_RESOLUTION_4k_17_9:          return H4_RESOLUTION_4k_17_9;
    case GOPRO_RESOLUTION_4k_16_9:          return H4_RESOLUTION_4k;
    case GOPRO_RESOLUTION_4k_SUPERVIEW:     return H4_RESOLUTION_4k_SUPERVIEW;
    case GOPRO_RESOLUTION_2_7k_17_9:        return H4_RESOLUTION_2_7k_17_9;
    case GOPRO_RESOLUTION_2_7k_16_9:        return H4_RESOLUTION_2_7k_16_9;
    case GOPRO_RESOLUTION_2_7k_SUPERVIEW:   return H4_RESOLUTION_2_7k_SUPERVIEW;
    case GOPRO_RESOLUTION_2_7k_4_3:         return H4_RESOLUTION_2_7k_4_3;
    case GOPRO_RESOLUTION_1440p:            return H4_RESOLUTION_1440p;
    case GOPRO_RESOLUTION_1080p_SUPERVIEW:  return H4_RESOLUTION_1080p_SUPERVIEW;
    case GOPRO_RESOLUTION_1080p:            return H4_RESOLUTION_1080p;
    case GOPRO_RESOLUTION_960p:             return H4_RESOLUTION_960p;
    case GOPRO_RESOLUTION_720p_SUPERVIEW:   return H4_RESOLUTION_720p_SUPERVIEW;
    case GOPRO_RESOLUTION_720p:             return H4_RESOLUTION_720p;
    case GOPRO_RESOLUTION_480p:             return H4_RESOLUTION_480p;
    close_switch_return_default(H4_RESOLUTION);
    }
}

////////////////////////////
// hero 4 framerate
////////////////////////////

GOPRO_FRAME_RATE h4_to_mav_framerate(uint8_t rate, bool *ok)
{
    default_true_open_switch(rate) {
    case H4_FRAMERATE_240fps:   return GOPRO_FRAME_RATE_240;
    case H4_FRAMERATE_120fps:   return GOPRO_FRAME_RATE_120;
    case H4_FRAMERATE_100fps:   return GOPRO_FRAME_RATE_100;
    case H4_FRAMERATE_90fps:    return GOPRO_FRAME_RATE_90;
    case H4_FRAMERATE_80fps:    return GOPRO_FRAME_RATE_80;
    case H4_FRAMERATE_60fps:    return GOPRO_FRAME_RATE_60;
    case H4_FRAMERATE_50fps:    return GOPRO_FRAME_RATE_50;
    case H4_FRAMERATE_48fps:    return GOPRO_FRAME_RATE_48;
    case H4_FRAMERATE_30fps:    return GOPRO_FRAME_RATE_30;
    case H4_FRAMERATE_25fps:    return GOPRO_FRAME_RATE_25;
    case H4_FRAMERATE_24fps:    return GOPRO_FRAME_RATE_24;
    case H4_FRAMERATE_15fps:    return GOPRO_FRAME_RATE_15;
    case H4_FRAMERATE_12_5fps:  return GOPRO_FRAME_RATE_12_5;
    close_switch_return_default(GOPRO_FRAME_RATE);
    }
}

H4_FRAMERATE mav_to_h4_framerate(uint8_t rate, bool *ok)
{
    default_true_open_switch(rate) {
    case GOPRO_FRAME_RATE_240:      return H4_FRAMERATE_240fps;
    case GOPRO_FRAME_RATE_120:      return H4_FRAMERATE_120fps;
    case GOPRO_FRAME_RATE_100:      return H4_FRAMERATE_100fps;
    case GOPRO_FRAME_RATE_90:       return H4_FRAMERATE_90fps;
    case GOPRO_FRAME_RATE_80:       return H4_FRAMERATE_80fps;
    case GOPRO_FRAME_RATE_60:       return H4_FRAMERATE_60fps;
    case GOPRO_FRAME_RATE_50:       return H4_FRAMERATE_50fps;
    case GOPRO_FRAME_RATE_48:       return H4_FRAMERATE_48fps;
    case GOPRO_FRAME_RATE_30:       return H4_FRAMERATE_30fps;
    case GOPRO_FRAME_RATE_25:       return H4_FRAMERATE_25fps;
    case GOPRO_FRAME_RATE_24:       return H4_FRAMERATE_24fps;
    case GOPRO_FRAME_RATE_15:       return H4_FRAMERATE_15fps;
    case GOPRO_FRAME_RATE_12_5:     return H4_FRAMERATE_12_5fps;
    close_switch_return_default(H4_FRAMERATE);
    }
}
