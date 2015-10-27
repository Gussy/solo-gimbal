#ifndef GMTIME_H
#define GMTIME_H

#include <time.h>

struct tm *gmtime_r(const time_t *timer, struct tm *result);

#endif // GMTIME_H
