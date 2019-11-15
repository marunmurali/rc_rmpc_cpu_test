#ifndef CAR_H
#define CAR_H

/**
* Created by: TRAN Anh Tuan 2019/09/06
*/

/* The Point class Header (Point.h) */

// Point class declaration
class Car {

private:
	double x;
	double y;
	double yaw;
	double vx;
	double vy;
	
public:
	Car(double x=0.0, double y = 0.0, double yaw = 0.0, double vx = 0.0, double vy = 0.0);
	
	double getX() const;
	double getY() const;
	double getYaw() const;
	void setX(double x);
	void setY(double y);
	void setYaw(double yaw);
	
	double getVx() const;
	double getVy() const;
	void setVx(double vx);
	void setVy(double vy);
	
	void print() const;
	
};






#endif
