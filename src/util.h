#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

#define ABS(x)                 ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)       {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}

#define _180_DIV_PI         57.295779f
#define PI_DIV_180          0.017453292f

#define F_TO_I(x)  ( (x) >= 0.0f ? (int32_t)((x) + 0.5f) : (int32_t)((x) - 0.5f) )

#endif
