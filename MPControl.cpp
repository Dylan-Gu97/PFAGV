#include "MPControl.h"
MPControlAlgorithms::MPControlAlgorithms(double exc, double eyc, double epsi, double vr)
{
	//����ϵͳ����
	d_exc = exc;
	d_eyc = eyc;
	d_epsi = epsi;
	d_vr = vr;
	//�������ʱ�䣬������ڿ��Ը���ʱ�亯������õ���
	double SampleTime = 0.1;
	//����״̬����
	A << 1, 0, 0,
		0, 1, SampleTime* d_vr,
		0, 0, 1;
	//�����������
	B << SampleTime * d_vr, 0,
		0, 0,
		0, 1;
	//����״̬����Ȩ��
	Q << 1, 0, 0,
		0, 12, 0,
		0, 0, 2;
	//����״̬�����ն˾���
	F << 1, 0, 0,
		0, 12, 0,
		0, 0, 2;
	//��������Ȩ�ؾ���
	R << 1, 0,
		0, 1;
	//��ʼ״̬����
	E_K << d_exc, d_eyc, d_epsi;
}

void MPControlAlgorithms::MPCMatrix()
{
	MatrixXd M = MatrixXd::Zero((N + 1) * n, n);
	M << MatrixXd::Identity(n, n), MatrixXd::Zero(N * n, n);
	MatrixXd C = MatrixXd::Zero((N + 1) * n, N * p);
	MatrixXd tmp = MatrixXd::Identity(n, p);
	for (int i = 1; i < N; i++) {
		/*���嵱ǰ����*/
		int rows = i * n;
		/*���C����*/
		C.block(rows, 0, n, N * p) << tmp * B, C.block(rows - n, p, n, (N - 1) * p);
		/*���M����*/
		tmp = A * tmp;
		M.block(rows, 0, n, n) = tmp;
	}

	MatrixXd Q_bar_init = kroneckerProduct(MatrixXd::Identity(N, N), Q);
	MatrixXd Q_bar = MatrixXd::Zero(Q_bar_init.rows() + n, Q_bar_init.cols() + n);
	Q_bar.topLeftCorner(n * N, n * N) = Q_bar_init;
	Q_bar.bottomRightCorner(n, n) = F;
	MatrixXd R_bar = kroneckerProduct(MatrixXd::Identity(N, N), R);

	G = M.transpose() * Q_bar * M;
	E = C.transpose() * M;
	H = C.transpose() * Q_bar * C + R_bar;
}

MatrixXd MPControlAlgorithms::Prediction()
{
	MatrixXd U_k = MatrixXd::Zero(N * p, 1);
	quadprogpp::Matrix<double>G(N * p, N * p);
	for (int i = 0; i < N * p; i++) {
		for (int j = 0; j < N * p; j++) {
			G[i][j] = H(i, j);
		}
	}
	quadprogpp::Vector<double>g(N * p);
	VectorXd gEigen = E * E_K; // (N*p - n) * (3 - 1)
	for (int i = 0; i < N * p; i++) {
		g[i] = gEigen(i);
	}
	quadprogpp::Matrix<double>CE(N * p, 1);
	quadprogpp::Vector<double>ce(1);
	quadprogpp::Matrix<double>CI(N * p, N * p);
	quadprogpp::Vector<double>ci(N * p);
	quadprogpp::Vector<double>x(N * p);
	for (int i = 0; i < N * p; i++) {
		CE[i][0] = 0;
		ci[i] = 0;
		for (int j = 0; j < N * p; j++) {
			CI[i][j] = 0;
		}
	}

	quadprogpp::solve_quadprog(G, g, CE, ce, CI, ci, x);

	VectorXd u_k;

	for (int i = 0; i < p; i++) {
		u_k(i) = x[i];
	}
	return u_k;
}

void MPControlAlgorithms::MPControlOutput(std::vector<double>& output)
{
	MPCMatrix();
	MatrixXd u_k = Prediction();
	double* data = u_k.data();
	for (int i = 0; i < p * n; i++) {
		output[i] = *(data + i);
	}
}

