#ifndef SENSING_H
#define SENSING_H

#include "Car.h"
#include "MathTools.h"
/* =====================================================================
 * Using derivative filter to obtain the velocity of the RC car 
 * =====================================================================
 */
void velocity_OptiTrack_simple(Car &car, double dt);


#endif
