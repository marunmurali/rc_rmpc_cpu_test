#include <stdio.h>
#include <math.h>
#include <Eigen/Core>
#include <stdlib.h>
#include <time.h>
#include "Jcpu.h"
#define RAD2DEG 180.0/M_PI
using namespace std;
using namespace Eigen;
using std::vector;
#include <defndel.h>

#include <random>
#include <vector>
#include<fstream>


double Jobs2CPU(double px, double py,double car_yaw,double car_input,double current_speed)
{
	py=1.5-py;
	//car_yaw=car_yaw;
	if(current_speed<.2)
	v=.2;
	else
	v=current_speed;
	//std::ofstream out("test.csv");

fprintf(stdout,"Current values: (x,y,yaw,ta,vel) = (%.2f, %.2f, %.2f, %.2f, %.2f) ",px, py, car_yaw*RAD2DEG, car_input*RAD2DEG,v);
std::cout<<std::endl;

	clock_t tStart = clock();
	random_device rnd;     // 非決定的な乱数生成器でシード生成機を生成
	mt19937 mt(rnd()); //  メルセンヌツイスターの32ビット版、引数は初期シード
	std::uniform_int_distribution<> norm(-99, 99);     // [0, 99] 範囲の一様乱数 int real
	//normal_distribution<> norm(0.0, 1.0);       // 平均0, 分散値1の正規分布
												//uniform_real_distribution<> rand2(-2, 2);

	for (int i = 0; i < cos_ckr2; ++i) {
		for (int j = 0; j < Ns; ++j) {
			usp[i][j] =  (float)norm(mt)/2000; //for test
			//usp[i][j] = norm(mt)*pi / 180;
		}
	}

	/// T transpose ‚Ì¶¬
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < N; ++j) {
			if (i == 0) {
				kij = 1 / sqrt(2.0);
			}
			else {
				kij = 1.0;
			}
			Tij_t[j][i] = sqrt(2.0 / N)*kij*cos((i - 1)*(j - 0.5)*pi / N);
		}
	}
	/// inverse transf
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < Ns; ++j) {
			double tem_calc2 = 0.0;
			for (int k = 0; k < N; ++k) {
				if (usp[k][j] != 0) {
					tem_calc2 = tem_calc2 + Tij_t[i][k] * usp[k][j];
				}
				else {
					break;
				}
			}
			usp_newt[i][j] = tem_calc2;
		}
	}
	// transpose
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < Ns; ++j) {
			uspnew[j][i] = usp_newt[i][j];
		}
	}
	//// —£ŽUƒRƒTƒCƒ“•ÏŠ· end

	// initialisation
	/*for (int j = 0; j < Ns; ++j){//for initialising first input as 0
		uspnew[j][0]  = car_input;
	}*/
	for (int j = 0; j < Ns; ++j) {
		xpos[j][0] = px;
		ypos[j][0] = py;
		theta[j][0] = car_yaw;
	}

	//for printing
	for (int i = 0; i < Ns; ++i) {
		for (int j = 0; j < N; ++j) {
			if(j==0) {
				u[i][j]=car_input;
				utmp=u[i][j];
			}
			else
			{
				u[i][j] = utmp+ uspnew[i][j];
	utmp=u[i][j];
}

			if (u[i][j] > imax)
			{
				u[i][j] = imax;
			}
			else if (u[i][j] < -imax)
			{
				u[i][j] = -imax;
			}
		utmp=u[i][j];
		}
	}

double p1 = 1 - (m*(lf*kf - lr*kr)*v*v) / (2 * l*l*kf*kr);
double p2 = 1 - (m*lf*v*v) / (2 * l*lr*kr);
double b1 = (p2*lr*v) / (l*p1);
double b2 = v / (l*p1);

	//extended xy theta matrix
	for (int i = 0; i < Ns; ++i) {
		for (int j = 1; j < N; ++j) {
			double ta = u[i][j];
			double beta = b1*ta/v;
			double rt = b2*ta;
			double rho = p1*l / ta;
			xpos[i][j] = xpos[i][j - 1] + 2 * rho*sin(.5*rt*dt)*cos((.5*rt*dt) + beta + theta[i][j - 1]);
			ypos[i][j] = ypos[i][j - 1] + 2 * rho*sin(.5*rt*dt)*sin((.5*rt*dt) + beta + theta[i][j - 1]);
			theta[i][j] = theta[i][j - 1] + rt*dt;
		}
	}

	//cost calculation
	//multipliers
	double dist1 = sqrt(pow((px - obs1[0] + disteff), 2.0) + pow(py - obs1[1], 2.0));
	double dist2 = sqrt(pow((px - obs2[0] + disteff), 2.0) + pow(py - obs2[1], 2.0));
	if (dist1 > tem) {
		dist1 = tem;
	}
	if (dist2 > tem) {
		dist2 = tem;
	}
	double eff1 = dist1 / tem;
	double eff2 = dist2 / tem;
	double sumeff = 2 + eff1 * eff2 - eff1 - eff2;
//cost
	for (int i = 0; i < Ns; ++i) {
		cost1[i] = 0;
		cost2[i] = 0;
		cost4[i] = 0;
		avoidobs1[i] = 0;
		avoidobs2[i] = 0;
		for (int j = 0; j < N; ++j) {
			cost1[i] = cost1[i] + pow(ypos[i][j], 2) + pow(theta[i][j], 2);
			if (j > 0) {
				cost2[i] = cost2[i] + pow((u[i][j] - u[i][j - 1]), 2);
			}
			if (ypos[i][j] > ywl || ypos[i][j] < ywr)
			{
				logcost = 10000;
			}
			else
			{
				logcost = log(abs(ywl)) + log(abs(ywr)) - log(ywl - ypos[i][j]) - log(ypos[i][j] - ywr);
			}
			if (isnan(logcost))
			{
				logcost = 10000;
			}
			cost4[i] = cost4[i] + logcost;
			double ao1t = pow((xpos[i][j] - obs1[0]), 2.0) / (reca * reca) + pow((ypos[i][j] - obs1[1]), 2.0) / (recb * recb);
			double ao2t = pow((xpos[i][j] - obs2[0]), 2.0) / (reca * reca) + pow((ypos[i][j] - obs2[1]), 2.0) / (recb * recb);

		/*if (avoidobs1[i] >= 1 && avoidobs2[i] >= 1) {*/
			avoidobs1[i] += c * exp(-ao1t);
			avoidobs2[i] += c * exp(-ao2t);
		/*}
		else {
			ao1t = 10000;
			ao2t = 10000;
			break;
		}*/
	}
	cost3 = pow(ypos[i][N - 1], 2) + pow(theta[i][N - 1], 2);
	//costobs[i] = avoidobs1[i] + avoidobs2[i];




	cost[i] = (eff1*eff2*(Q*cost1[i]+R*cost2[i]+ sf*cost3)+(1 - eff1)*P*avoidobs1[i]/Ns +(1 - eff2)*P*avoidobs2[i]/Ns)/sumeff+Rw*cost4[i];
	//Jsum[i] = (eff1 * eff2 * sf * J2[i] + eff1 * eff2 * Q * J1[i] + (1 - eff1) * P * Pot1[i] / Ns + (1 - eff2) * P * Pot2[i] / Ns + eff1 * eff2 * R * usum[i]) / sumeff + Rw * Potwall[i];
	if(isnan(cost[i]))
		{
		std::cout<<"cost is nan"<< std::endl;
		cost[i]=10000000;
		}
	}

	//mincost
	double minJsum = cost[0];
minindex = 0;
	for (int ij = 1; ij < Ns; ++ij) {
		if (minJsum > cost[ij]) {
			minJsum = cost[ij];
			minindex = ij;
		}
	}
std::cout<<"index of min is"<< minindex<<std::endl;



return u[minindex][2];


	vector< vector<double> > ().swap(u);
	vector< vector<double> >().swap(usp);
	vector< vector<double> >().swap(uspnew);
	vector< vector<double> >().swap(usp_newt);
	vector< vector<double> >().swap(Tij_t);
	vector< vector<double> >().swap(xpos);
	vector< vector<double> >().swap(ypos);
	vector< vector<double> >().swap(theta);

}
