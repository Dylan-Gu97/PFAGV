#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace Eigen;
class SlidingModeControlAlgorithms
{
private:
	// ����Ŀ��
	double x_d;
	double dx_d; 
	double ddx_d;
	double psi_d; 
	double dpsi_d; 
	double ddpsi_d;
	double theta_d; 
	double dtheta_d; 
	double ddtheta_d;
	// ״̬����
	double x;
	double psi;
	double theta;
	double dx;
	double dpsi;
	double dtheta;

	Vector3d qd;
	Vector3d dqd;
	Vector3d ddqd;
	Vector3d q;
	Vector3d dq;
	Matrix3d Lam;
	Vector3d e;
	Vector3d de;
	Vector3d dqr;
	Vector3d ddqr;
	double dxr;
	double ddxr;
	double dthetar;
	double ddthetar;
	double dpsir;
	double ddpsir;
	Vector3d s; //��ģ��
	// ϵͳ����ѧ���̲���
	VectorXd p_real;
	VectorXd p;
	VectorXd p_max;
	double alpha;
	double beta;
	double gamma;
	double epsi;
	double eta;
	double cal;
	double g = 9.8;
	double r = 0.0075;
	double d = 0.7;
	Matrix3d M;
	Matrix3d C;
	Matrix3d D;
	Matrix<double, 3, 2> B;
	Vector3d G;
	Matrix<double, 3, 6> Y;
	Matrix<double, 3, 6> Y_max;
	Vector3d k;

	Vector3d sats;

public:
	SlidingModeControlAlgorithms(std::vector<double>& uVector);
	void StateVariableDef();
	void SlidingModePlaneDef();
	void DynamicEqCoeffDef();
	Vector3d Talcal();
	void ControlOutput(std::vector<double>& output);

	template <typename T> int sgn(T val);
	MatrixXd pinv(MatrixXd  A);
};

