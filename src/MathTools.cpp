#include "MathTools.h"


/* =====================================================================
 *  Derivative filter (FIXED Tf)
 * Calculate the derivative of signal u considering low-pass filter
 * s/(Tf*s+1)*U(s) = Y(s)
 * =====================================================================
 */
void derivative_filter(double u, double dt, double &x, double &du){
	
	double Tf = 0.1;
	
	// Calculate the value of derivative of u
	du = -1/Tf*x+1/Tf*u;
		
	// Update x
	x += dt * (-1/Tf*x + 1/Tf*u);

}

/* =====================================================================
 *  Derivative filter (User-defined Tf) 
 * Calculate the derivative of signal u considering low-pass filter
 * s/(Tf*s+1)*U(s) = Y(s)
 * =====================================================================
 */
void derivative_filter(double u, double dt, double Tf, double &x, double &du){
	
	// Calculate the value of derivative of u
	du = -1/Tf*x+1/Tf*u;
		
	// Update x
	x += dt * (-1/Tf*x + 1/Tf*u);

}





/* =====================================================================
 * Modulus function
 * =====================================================================
 */
 
//template <typename T>
//T mod(T a, T b) { return a - b * floor(a/b); }
