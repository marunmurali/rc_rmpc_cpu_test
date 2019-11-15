#include "Sensing.h"
#include <iostream>

/* =====================================================================
 * Using derivative filter to obtain the velocity of the RC car 
 * =====================================================================
 */
void velocity_OptiTrack_simple(Car &car, double dt){
	static double x_vx = 0.0;
	static double x_vy = 0.0;
	
	double vx=0.0;
	double vy=0.0;
	
	// Calculate the velocities vx, vy
	derivative_filter(car.getX(),dt, x_vx, vx);
	derivative_filter(car.getY(),dt, x_vy, vy);
	//fprintf(stdout,"vx=%.2f\n", vx);
	//fprintf(stdout,"vy=%.2f\n", vy);
	car.setVx(vx);
	car.setVy(vy);
	//car.print();
}
	
