#ifndef DEFNDEL_H
#define DEFNDEL_H
#pragma once
#include <iostream>
#include <vector>

double kf =19.7, kr = 12.2, lf = .14784, lr = .10716,l = .255, m = 2.0302, acc, x_tmp, utmp = 0;//
//kf =0.0197, kr = 0.0122
double u_vir = 0;// virtual of tire_angle
double dt = 0.05, T = 30 / dt;//dt:�������� T:not used
int N = 30, Ns = 500, vkmh = 3.6;// N:�\���z���C�Y������ Ns:�T���v���n����
double v;// = vkmh / 3.6;//���x m/s
//double v1;

					  //�ړ��\�����f���ɕK�v�Ȍv�Z
//double p1 = 1 - (m*(lf*kf - lr*kr)*v*v) / (2 * l*l*kf*kr);
//double p2 = 1 - (m*lf*v*v) / (2 * l*lr*kr);
//double b1 = (p2*lr*v) / (l*p1);
//double b2 = v / (l*p1);

double uold;
double imax = 0.34;//limit of steering angle in rads
int minindex;
double logcost;
int cos_ckr2 = 15;//less than N
double kij;
double temparray[20][50];
double tem=.5;
double disteff=.1;

vector< vector<double> > u(Ns, vector<double>(N));
vector< vector<double> > uspnew(Ns, vector<double>(N));
vector< vector<double> > usp_newt(N, vector<double>(Ns));
vector< vector<double> > Tij_t(N, vector<double>(N));
vector< vector<double> > usp(N, vector<double>(Ns));
vector< vector<double> > xpos(Ns, vector<double>(N));
vector< vector<double> > ypos(Ns, vector<double>(N));
vector< vector<double> > theta(Ns, vector<double>(N));
//for cost function
vector<double> usum(Ns);//���͍��� delta_u3
vector<double> cost1(Ns);
vector<double> cost2(Ns);
vector<double> avoidobs1(Ns);
vector<double> avoidobs2(Ns);
vector<double> costobs(Ns);
double cost3 = 0;
vector<double> cost4(Ns);

vector<double> cost(Ns);//J(t)
vector<double> Pot1(Ns);//���Q���|�e���V����
vector<double> Pot2(Ns);
vector<double> Pot3(Ns);
vector<double> Potwall(Ns);//�ǂ̃|�e���V����
vector<double> uback(Ns);
//�n�[�h�����̑ȉ~�̒��Z��
double reca = 0.7;
double recb = 0.2;
double recanew = 0.4;
//�ǂ̏���
double ywl = .4;
double ywr = -ywl;
//double ywr=-5.0;
double wallmin = 2 * log(ywl);
//double wallmin=log(3.0)+log(5.0);

const double pi = 3.141592654;
double umin = -2 * pi / 180;
double umax = 2 * pi / 180;

//�]���֐��̊e�p�����[�^�̐ݒ�
int sf = 1000, Q = 1, R = 500, P = 7000, c = 10, Rt = 0, Rw = 3;//P is Robs

// try
//int sf=1, Q=100, R=3000, P=3000, c=10, Rt=0, Rw=5;

//���Q���̈ʒu
double obs1[2] = { .8,0.05 };
double obs2[2] = { 3,-0.05 };
double obs3[2] = { 30,0.85 };

double tem_calc1, tem_calc2;
#endif
