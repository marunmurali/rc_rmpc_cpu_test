/**
 * Created by: TRAN Anh Tuan 2019/09/06
 * 
*/

#include "Car.h"
#include <iostream>
#include <iomanip>
/* Class Car
 * 
 * 
 * 
 */
 
// Constructor
Car::Car(double _x, double _y, double _yaw, double _vx, double _vy) : x(_x), y(_y), yaw(_yaw), vx(_vx) , vy(_vy) {};

// Getter
double Car::getX() const { return x; }
double Car::getY() const {	return y; }
double Car::getYaw() const {return yaw; }

double Car::getVx() const { return vx; }
double Car::getVy() const { return vy; }

//Setter
void Car::setX(double _x) { this->x = _x; }
void Car::setY(double _y) { this->y = _y; }
void Car::setYaw(double _yaw) {this->yaw = _yaw; }

void Car::setVx(double _vx) { this->vx = _vx; }
void Car::setVy(double _vy) { this->vy = _vy; }

// Print
void Car::print() const {
	std::cout << "Car @ (x,y,yaw,vx,vy) = ("<<std::fixed<<std::setprecision(2)<<x<<","<<y<<","<<yaw<<", "<<vx<<","<<vy<<")"<<std::endl;
} 
