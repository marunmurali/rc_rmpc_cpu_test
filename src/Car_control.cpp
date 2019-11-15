#include "Car_control.h"
#include "math.h"
#include <iostream>
#include "MathTools.h"

/**
* Created by: TRAN Anh Tuan 2019/09/06
*/


/* =====================================================================
 * Speed PID controller
 * =====================================================================
 */
double car_control_speed_PID(double current_speed, double speed_ref, double dt, double Tf){
	double Kp = 0;
	double Ki = 1;
	double Kd = 1; 
	double speed_error_integrator_max = 50;
	double speed_error_integrator_min = -50;
	static double speed_error_integrator = 0.0;
	static double x_speed_error_derivative = 0.0;
	
	double speed_error = 0.0;
	double speed_error_derivative = 0.0;
	double a, b;
	
	double Kp_max = 100;
	double speed_error_max = 2;
	
	double Kp_min = 10;
	double speed_error_min = 0.1;
	
	speed_error = speed_ref - current_speed;

	// Variable Kp gain: Kp = a * speed_error + b;
	// Kp = Kp_max when speed_error is small
	// Kp = Kp_min when speed_error is large
	if (abs(speed_error) > speed_error_max) {
		Kp = Kp_min;
	}
	else if (abs(speed_error)  < speed_error_min) {
		Kp = Kp_max;
	}
	else {
		a = (Kp_min - Kp_max) / (speed_error_max - speed_error_min);
		b = Kp_max - a * speed_error_min;
		Kp =  a * speed_error + b;
	}
		
	//fprintf(stdout,"speed_error %.2f, Kp %.2f\n", speed_error, Kp);
	
	// Calculate speed_error_derivative using derivative filter		
	derivative_filter(speed_error, dt, Tf, x_speed_error_derivative,speed_error_derivative);
	
	// Integrator
	speed_error_integrator += speed_error;
	
	//Anti-windup
	speed_error_integrator = limit_function(speed_error_integrator,speed_error_integrator_max,speed_error_integrator_min);
	
	double PID_output;
	PID_output = Kp*speed_error + Ki*speed_error_integrator + Kd*speed_error_derivative;
	PID_output = limit_function(PID_output, 400, 0);	
	
	return PID_output;
}


/* =====================================================================
 * Lane keeping PID controller
 * =====================================================================
 */
double car_control_lateral(double pos_y, double pos_y_ref, double v_y)
{
	// Not finished yet!!!!
	double Kp = 0.0;
	double Kd = 0.0;
	//float Kffw = 0.0; // feedforward gain
	double steer_pwm;
	
	steer_pwm = Kp*(pos_y - pos_y_ref) + Kd*v_y;
	return steer_pwm;
	
}
	


