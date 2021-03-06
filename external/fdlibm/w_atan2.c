
/* @(#)w_atan2.c 1.3 95/01/18 */


#include "fdlibm.h"


#ifdef __STDC__
	double ieee_atan2(double y, double x)	/* wrapper atan2 */
#else
	double ieee_atan2(y,x)			/* wrapper atan2 */
	double y,x;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_atan2(y,x);
#else
	double z;
	z = __ieee754_atan2(y,x);
	if(_LIB_VERSION == _IEEE_||ieee_isnan(x)||ieee_isnan(y)) return z;
	if(x==0.0&&y==0.0) {
	        return __kernel_standard(y,x,3); /* ieee_atan2(+-0,+-0) */
	} else
	    return z;
#endif
}
