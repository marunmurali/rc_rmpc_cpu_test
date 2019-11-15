#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include "Car.h"
/* =====================================================================
 * Compute time derivative with low-pass filter
 * =====================================================================
 */
void derivative_filter(double u, double dt, double &x, double &du);
void derivative_filter(double u, double dt, double Tf, double &x, double &du);


/* =====================================================================
 * Modulus function
 * =====================================================================
 */
 
//template <typename T>
//T mod(T a, T b);




#endif
