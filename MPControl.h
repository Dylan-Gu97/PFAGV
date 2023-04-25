#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/src/KroneckerProduct/KroneckerTensorProduct.h>
#include "QuadProg++.hh"
#include <vector>
using namespace Eigen;

class MPControlAlgorithms
{
private:
	double d_exc; // 车辆与预定行驶轨道的前向距离差距
	double d_eyc; // 车辆与预定形式轨道的纵向距离差距
	double d_epsi; // 车辆与预定轨道的偏航角差距
	double d_vr; // 车辆预定的前向速度，积分可得前向预定距离
	Matrix3d A; //状态矩阵
	Matrix<double, 3, 2>B; //输入矩阵
	Matrix3d Q;
	Matrix3d F; // Q为状态变量权重矩阵，F为状态变量终端权重矩阵
	Matrix2d R; //输入权重矩阵
	Vector3d E_K; //初始状态变量
	Vector2d U_K; //输出行向量
	MatrixXd G;
	MatrixXd E;
	MatrixXd H;
	int N = 10; //预测区间
	int n = A.rows();
	int p = B.cols();

public:
	MPControlAlgorithms(double exc, double eyc, double epsi, double vr);
	void MPCMatrix();
	MatrixXd Prediction();
	void MPControlOutput(std::vector<double>& output);
};

