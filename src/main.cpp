#include "ros/ros.h"
#include <time.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "Jcpu.h"
#include <stdio.h>
#include <iostream>
#include "math.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "iostream"
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <custom_pkg/control_signal.h>
#include <custom_pkg/log_sig.h>
//#include "FrenetCoordinate.h"
#include "Car.h"
#include "MathTools.h"
#include "Sensing.h"
//#include "Car_control.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <fstream>
#define RAD2DEG 180.0/M_PI

using namespace Eigen;
int *testmain(int num, int threads);
int size=10;
std_msgs::Float32 msg_time;
float angle_Optitrack_x, angle_Optitrack_y, angle_Optitrack_z;
float pos_x, pos_y, pos_z;
//double imax=0.2967;//17 degrees in radian
float car_input;

void size_Callback(const std_msgs::Int32& msg)
{
	size=msg.data;
	if(size<1){size=1;}
	if(size>100){size=100;}
}
//=====================================================================
 //* Receive data from Motion Capture topic * =====================================================================
 //*/
void chatterCallback_Optitrack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{



    pos_x = msg->pose.position.x;
    pos_y = msg->pose.position.y;	// Convert ROS frame to IMU frame
    pos_z = msg->pose.position.z;	// Convert ROS frame to IMU frame

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation,quat);

    double roll_opti, pitch_opti, yaw_opti;

    tf::Matrix3x3(quat).getRPY(roll_opti,pitch_opti,yaw_opti);

    angle_Optitrack_x = pitch_opti;		// In Ground Coordinate systemg
    angle_Optitrack_y = roll_opti;
    angle_Optitrack_z = yaw_opti;
    /*
    fprintf(stdout,"Pos. X = %.2f, Pos. Y = %.2f, Pos. Z = %.2f \n", pos_x, pos_y, pos_z);
    fprintf(stdout,"Angle (Opti) X = %.2f, Angle (Opti) Y = %.2f, Angle (Opti) Z = %.2f \n", angle_Optitrack_x, angle_Optitrack_y, angle_Optitrack_z);
    */
}

//=====================================================================
//* Main *
//====================================================================

int main(int argc, char **argv)
{
ros::init(argc, argv, "imu_receive");
ros::NodeHandle n;
ros::Subscriber sub1 = n.subscribe("/vrpn_client_node/RigidBody001/pose", 1000, chatterCallback_Optitrack);
ros::Publisher time_pub = n.advertise<std_msgs::Float32>("/time", 1);
ros::Subscriber size_sub = n.subscribe("/size", 100, size_Callback);
ros::Publisher pub = n.advertise<custom_pkg::control_signal>("/control_signal001", 50);
ros::Publisher publ = n.advertise<custom_pkg::log_sig>("/log_sig", 50);
custom_pkg::control_signal dat;
custom_pkg::log_sig logd;
ros::Rate loop_rate(100);	// 100 Hz
double speed_ref = 1.8;
double Tf = 0.2;	// Derivative filter's time constant
double steer_pwm_cmd = 0.0;
double speed_pwm_cmd = 0.0;
double lcurrent_speed = 0.0;
double lcar_input=0;
double dt = 0.01;
clock_t start, end;
double cpu_time_used;
int *p;
std::cout<<"Start!"<<std::endl;
double vx, vy;
double lpx, lpy, pz, lcar_yaw;
double angle_Opt_x,angle_Opt_y,angle_Opt_z;
Car car;

// START LOOOOOOOOOOOOP!
while(ros::ok())
{
start = clock();
angle_Opt_x = angle_Optitrack_x;
angle_Opt_y = angle_Optitrack_y;
angle_Opt_z = angle_Optitrack_z;
// Calculate velocity
// Note that the update frequency of pos_x, pos_y (which will be updated every time the motion capture system sends a new message)
// and the update frequency of car.setX, car.setY, etc. (which is determined by the ros::rate) are different
        car.setX(pos_x);
        car.setY(pos_y);
        car.setYaw(angle_Optitrack_z);
        velocity_OptiTrack_simple(car,dt);
        //car.print();		// Print position and velocity of the car
        vx = car.getVx();
        vy = car.getVy();
        lcar_yaw = car.getYaw();
        lpx = car.getX();
        lpy = car.getY();
        lcurrent_speed = sqrt(vx*vx+vy*vy);
// Calculate the car input
//::cout<<angle_Optitrack_z*RAD2DEG<<std::endl;

lcar_input=Jobs2CPU(lpy, lpx, lcar_yaw, lcar_input,lcurrent_speed);//0.0174533*

//send to car
dat.header.stamp=ros::Time::now();
if(lpy>4)
{
dat.accel=00;
}
else
{
dat.accel=5;//number for speed pwm
}
dat.steer=lcar_input*RAD2DEG*15.84;//number for steering pwm
dat.status=1;
pub.publish(dat);
end = clock();
//timeused
cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
//logsignal
logd.header.stamp=ros::Time::now();
logd.accel=dat.accel;//number for steering pwm
logd.steer=dat.steer;
logd.xpos=lpy;
logd.ypos=1.5-lpx;
logd.ta=lcar_input;
logd.yaw=lcar_yaw;
logd.vel=lcurrent_speed;
logd.status=1;
logd.comptime=cpu_time_used;
publ.publish(logd);
std::cout<<cpu_time_used<<std::endl;
//msg_time.data=cpu_time_used;
//time_pub.publish(msg_time);
ros::spinOnce();
loop_rate.sleep();
}
return 0;
}
