#include <math.h>

#ifndef FLOAT_CHECKS_H_
#define FLOAT_CHECKS_H_

#ifndef isnan
# define isnan(x) \
		(sizeof (x) == sizeof (long double) ? isnan_ld (x) \
				: sizeof (x) == sizeof (double) ? isnan_d (x) \
						: isnan_f (x))
static inline int isnan_f  (float       x) { return x != x; }
static inline int isnan_d  (double      x) { return x != x; }
static inline int isnan_ld (long double x) { return x != x; }
#endif

#ifndef isinf
# define isinf(x) \
		(sizeof (x) == sizeof (long double) ? isinf_ld (x) \
				: sizeof (x) == sizeof (double) ? isinf_d (x) \
						: isinf_f (x))
static inline int isinf_f  (float       x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_d  (double      x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_ld (long double x)
{ return !isnan (x) && isnan (x - x); }
#endif

#ifndef isnumber
# define isnumber(x) \
		(!isnan(x) && !isinf(x))
#endif

#endif
