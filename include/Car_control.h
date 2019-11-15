#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

/**
* Created by: TRAN Anh Tuan 2019/09/06
*/


/* =====================================================================
 * Modulus function
 * =====================================================================
 */
 
//template <typename T>
//T mod(T a, T b) {
	//T temp; 
	//if (a/b>0) {
		//temp = a-b*floor(a/b);
	//}
	//else {
		//temp = a-b*ceil(a/b);
	//}
	
	//return temp; 
//}
template <typename T>
T mod(T a, T b) { return a-b*round(a/b); }


/* =====================================================================
 * Limit function
 * Set the maximum and minimum values
 * =====================================================================
 */
template <typename T>
T limit_function(T value, T value_max, T value_min){
	if (value >= value_max){
		value = value_max;
	} else if (value <= value_min) {
		value = value_min;
	}
	return value;
}

/* =====================================================================
 * Speed PID controller
 * =====================================================================
 */
double car_control_speed_PID(double current_speed, double speed_ref, double dt, double Tf, double flag){
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
		
	fprintf(stdout,"speed_error %.2f, Kp %.2f\n", speed_error, Kp);
	
	// Calculate speed_error_derivative using derivative filter		
	derivative_filter(speed_error, dt, Tf, x_speed_error_derivative,speed_error_derivative);
	
	// Integrator
	speed_error_integrator += flag*speed_error;
	
	//Anti-windup
	speed_error_integrator = limit_function(speed_error_integrator,speed_error_integrator_max,speed_error_integrator_min);
	
	return Kp*speed_error + Ki*speed_error_integrator + Kd*speed_error_derivative;
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
	
/* =====================================================================
 * Lane keeping State Feedback controller 
 * Created by: H. Okuda
 * =====================================================================
 */
struct StateFBControlParameter
{
	double PFLLook;
	double PFSpeed;

	double PFKspeed;
	double PFKangle;

	double SFBK1;
	double SFBK2;
	double SFBK3;
	double SFBK4;

	double SFBLLook;

	StateFBControlParameter()
	{
		PFLLook = 10;
		PFSpeed = 10;
		PFKspeed = -2.0;
		PFKangle = -1.0;

		SFBK1 = 1000;
		SFBK2 = 100;
		SFBK3 = 1000;
		SFBK4 = 50;

		SFBLLook = 0.6;
	}

	void ReadFromMap(std::map<std::string, double> param)
	{
		TryGetVal(param, "Speed", PFSpeed);
		TryGetVal(param, "PFkspeed", PFKspeed);
		TryGetVal(param, "PFkangle", PFKangle);
		TryGetVal(param, "PFLLook", PFLLook);

		TryGetVal(param, "k1", SFBK1);
		TryGetVal(param, "k2", SFBK2);
		TryGetVal(param, "k3", SFBK3);
		TryGetVal(param, "k4", SFBK4);

		TryGetVal(param, "LLook", SFBLLook);
	}

	void TryGetVal(std::map<std::string, double> &param, std::string key, double &val)
	{
		if (param.count(key) == 1)
		{
			val = param[key];
		}
		else {
			
		}
	}
};



#endif
