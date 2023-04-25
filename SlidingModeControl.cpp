#include "SlidingModeControl.h"

SlidingModeControlAlgorithms::SlidingModeControlAlgorithms(std::vector<double>& uVector)
{
	// 控制目标初始化
	x_d = uVector[0];
	dx_d = uVector[1];
	ddx_d = uVector[2];
	psi_d = uVector[3];
	dpsi_d = uVector[4];
	ddpsi_d = uVector[5];
	theta_d = uVector[6];
	dtheta_d = uVector[7];
	ddtheta_d = uVector[8];

	// 状态变量初始化
	x = uVector[9];
	dx = uVector[10];
	psi = uVector[11];
	dpsi = uVector[12];
	theta = uVector[13];
	dtheta = uVector[14];

	// 系统动力学方程参数设定
	p << 1973, 480, 1600, 46.62, -1560, 2000;
	p = 0.5 * p_real;
	alpha = p(1);
	beta = p(2);
	gamma = p(3);
	epsi = p(4);
	eta = p(5);
	cal = p(6);
}

void SlidingModeControlAlgorithms::StateVariableDef()
{
	qd << x_d, theta_d, psi_d;
	dqd << dx_d, dtheta_d, dpsi_d;
	ddqd << ddx_d, ddtheta_d, ddpsi_d;
	q << x, theta, psi;
	dq << dx, dtheta, dpsi;
	Lam << 2, 0, 0,
		   0, 2, 0,
		   0, 0, 2;
	e = qd - q;
	de = dqd - dq;
	dqr = dqd - Lam * e;
	ddqr = ddqd + Lam * e;
	dxr = dqr(1);
	dthetar = dqr(2);
	dpsir = dqr(3);
	ddxr = ddqr(1);
	ddthetar = ddqr(2);
	ddpsir = ddqr(3);
}

void SlidingModeControlAlgorithms::SlidingModePlaneDef()
{
	s = de + Lam * e;
	// sign符号函数优化
	double delta = 0.05;
	double kk = static_cast<double>(1 / delta);
	for (int i = 0; i < s.rows(); i++) {
		if (fabs(s(i)) > delta)
			sats(i) = sgn(s(i));
		else
			sats(i) = kk * s(i);
	}
}

void SlidingModeControlAlgorithms::DynamicEqCoeffDef()
{
	M << alpha, beta* cos(theta), 0,
		 beta* cos(theta), gamma, 0,
		 0, 0, epsi - eta * pow(sin(theta),2);
	C << 0, -beta * dtheta * sin(theta), -beta * dpsi * sin(theta),
		 0, 0, eta* dpsi* sin(theta)* cos(theta),
		 beta* dpsi* sin(theta), -eta * dpsi * sin(theta) * cos(theta), -eta * dtheta * sin(theta) * cos(theta);
	D << 2 * cal / pow(r, 2), -2 * cal / r, 0,
		-2 * cal / r, 2 * cal, 0,
		 0, 0, (pow(d, 2) / (2 * pow(r, 2)))* cal;
	B << 1 / r, 1 / r,
		-1, -1,
		-d / (2 * r), d / (2 * r);
	G << 0, -beta * g * sin(theta), 0;
	Y << ddxr, (ddthetar * cos(theta) - dtheta * dthetar * sin(theta) - dpsi * sin(theta) * dpsir), 0, 0, 0, (2 * dxr / pow(r, 2) - 2 * dthetar / r),
		 0, (cos(theta) * ddxr - g * sin(theta)), ddthetar, 0, (dpsi * sin(theta) * cos(theta) * dpsir), (-2 * dxr / r + 2 * dthetar),
		 0, dpsi* sin(theta)* dxr, 0, ddpsir, (-pow(sin(theta), 2) * ddpsir - dpsi * sin(theta) * cos(theta) * dthetar - dtheta * sin(theta) * cos(theta) * dpsir), (pow(d, 2) * dpsir) / (2 * pow(r, 2));

	double coeff1 = 0.1;
	ArrayXXd Y_max_array(Y_max.rows(), Y_max.cols());
	Y_max_array = Y_max.array().abs();
	Y_max = Y_max_array.matrix() + MatrixXd::Ones(Y.rows(), Y.cols()) * coeff1;

	double coeff2 = 0.5;
	MatrixXd p_max_real(p_max.rows(), p_max.cols());
	p_max_real = p_real - p;
	ArrayXXd p_max_array(p_max.rows(), p_max.cols());
	p_max_array = p_max_real.array().abs();
	p_max = p_max_array.matrix() + MatrixXd::Ones(p_max.rows(), p_max.cols()) * coeff2;
	
	k = Y_max * p_max;

}

Vector3d SlidingModeControlAlgorithms::Talcal()
{
	Matrix3d satsMatrix;
	satsMatrix << sats(1), 0, 0,
				  0, sats(2), 0,
				  0, 0, sats(3);
	Vector3d tals;
	tals = satsMatrix * k + s;
	return tals;
}

void SlidingModeControlAlgorithms::ControlOutput(std::vector<double>& output)
{
	Vector3d tal_real;
	Vector3d tals;
	StateVariableDef();
	SlidingModePlaneDef();
	DynamicEqCoeffDef();
	tals = Talcal();
	tal_real = M * ddqr + C * dqr + D * dqr + G + tals;
	MatrixXd tal(2, 1);
	tal = pinv(B) * tal_real;
	output.push_back(tal(0));
	output.push_back(tal(1));
}

MatrixXd SlidingModeControlAlgorithms::pinv(MatrixXd  A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	double  pinvtoler = 1.e-8; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = fmin(row, col);
	MatrixXd X = Eigen::MatrixXd::Zero(col, row);
	MatrixXd singularValues_inv = svd.singularValues();//奇异值
	MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

	return X;
}

template <typename T> 
int SlidingModeControlAlgorithms::sgn(T val) {
	return (T(0) < val) - (val - T(0));
}