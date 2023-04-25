#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/src/KroneckerProduct/KroneckerTensorProduct.h>
#include "QuadProg++.hh"
#include <vector>
using namespace Eigen;

class MPControlAlgorithms
{
private:
	double d_exc; // ������Ԥ����ʻ�����ǰ�������
	double d_eyc; // ������Ԥ����ʽ��������������
	double d_epsi; // ������Ԥ�������ƫ���ǲ��
	double d_vr; // ����Ԥ����ǰ���ٶȣ����ֿɵ�ǰ��Ԥ������
	Matrix3d A; //״̬����
	Matrix<double, 3, 2>B; //�������
	Matrix3d Q;
	Matrix3d F; // QΪ״̬����Ȩ�ؾ���FΪ״̬�����ն�Ȩ�ؾ���
	Matrix2d R; //����Ȩ�ؾ���
	Vector3d E_K; //��ʼ״̬����
	Vector2d U_K; //���������
	MatrixXd G;
	MatrixXd E;
	MatrixXd H;
	int N = 10; //Ԥ������
	int n = A.rows();
	int p = B.cols();

public:
	MPControlAlgorithms(double exc, double eyc, double epsi, double vr);
	void MPCMatrix();
	MatrixXd Prediction();
	void MPControlOutput(std::vector<double>& output);
};

