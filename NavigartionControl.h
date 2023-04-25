#pragma once
#include "AlgorithmsInterface.h"
#include "MPControl.h"
#include "SlidingModeControl.h"
#include "SensorInterface.h"
#include "magNavSensor.h"
#include <vector>
using namespace std;

/* ·��׷�ٿ���ģ��
 * ģ�����Ŀ�꣺1����Сx������2����Сy������3����С�����psi������4����С������theta�������
 * ģ�����룺1������Ŀ�꣬2����ǰ״̬
 * ����Ŀ���������1��x_r��2��y_r 3��psi_r = 0 4��theta_r = 0
 * ��ǰ״̬��������1��x��dx��2��y��dy��3��psi��dpsi��
 * ģ��������������������أ�Tal1��Tal2��
 */

 /* ���ȳ���ʹ��MPC�ٶȿ��ƣ�
  * ��������ģ��ֻ��Ҫʹ��MPControlAlgorithmsģ�鼴�ɣ��������SlidingModeControlAlgorithms��
  * ����Ҫmpc����ٶ�Ŀ�꣬��ģ�����������Ŀ��
  * ������㷨��ʵ��Ϊ���յĵ��ýӿڣ����߱�main�������ã����߱�ĳ���̵߳��ã���ˣ����������κ����ݣ�û������
  * ������㷨���Ӧ�ľ���ʵ��������Ҫ��������������������ĸ�������������������ĸ����
  */

  // ����Ŀ��ṹ��
struct ControlTarget
{
	double x_r;
	double y_r;
	double psi_r;
};

// ״̬�����ṹ��
struct StateVar
{
	double x;
	double dx;
	double y;
	double dy;
	double psi;
	double dpsi;
};

struct NavControlInput
{
	// MPC����
	double exc;
	double eyc;
	double epsi;
	double vr;
};

struct NavControlOutput
{
	//�������
	double TL;
	double TR;
};

class NavigartionControl :
	public AlgorithmsInterface
{
private:
	/*MPControlAlgorithms MPControl;*/
	/*SlidingModeControlAlgorithms SlidModeControl;*/
	ControlTarget d_control_target;
	StateVar d_state_var;
	NavControlInput d_nav_control_input;
	NavControlOutput d_nav_control_output;

	string COMPort;
	vector<Sensor2Control*> Sensor2ControlVector;
	vector<Control2Actuator*> Control2ActuatorVector;
	YawCalculate* mag;




public:
	NavigartionControl();
	~NavigartionControl();

	void ControlInput()
	{
		//����Ŀ������
		/*d_control_target.x_r = input[0];
		d_control_target.y_r = input[1];
		d_control_target.psi_r = input[2];*/

		//״̬�������
		/*d_state_var.x = input[3];
		d_state_var.dx = input[4];
		d_state_var.y = input[5];
		d_state_var.dy = input[6];
		d_state_var.psi = input[7];
		d_state_var.dpsi = input[8];*/
	}

	void ControlAction()
	{
		d_nav_control_input.exc = d_state_var.x - d_control_target.x_r;
		d_nav_control_input.eyc = d_state_var.y - d_control_target.y_r;
		d_nav_control_input.epsi = d_state_var.psi - d_control_target.psi_r;
		d_nav_control_input.vr = d_control_target.x_r;
		//MPC���
		vector<double> MPCoutput;
		double ev = MPCoutput.at(0);
		double ew = MPCoutput.at(1);
		//��ģ��������

		//��ģ�������

		/*MPControl(exc, eyc, epsi, vr);*/
		/*MPControl.MPControlOutput(MPCoutput);*/
	}


	void ControlOutput()
	{
		/*MPControl.MPControlOutput();*/
	}
};

