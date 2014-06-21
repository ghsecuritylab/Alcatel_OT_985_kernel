
/* @(#)w_cosh.c 1.3 95/01/18 */


#include "fdlibm.h"

#ifdef __STDC__
	double ieee_cosh(double x)		/* wrapper cosh */
#else
	double ieee_cosh(x)			/* wrapper cosh */
	double x;
#endif
{
#ifdef _IEEE_LIBM
	return __ieee754_cosh(x);
#else
	double z;
	z = __ieee754_cosh(x);
	if(_LIB_VERSION == _IEEE_ || ieee_isnan(x)) return z;
	if(ieee_fabs(x)>7.10475860073943863426e+02) {	
	        return __kernel_standard(x,x,5); /* cosh overflow */
	} else
	    return z;
#endif
}