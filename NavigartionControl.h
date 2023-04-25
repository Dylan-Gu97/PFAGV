#pragma once
#include "AlgorithmsInterface.h"
#include "MPControl.h"
#include "SlidingModeControl.h"
#include "SensorInterface.h"
#include "magNavSensor.h"
#include <vector>
using namespace std;

/* 路径追踪控制模块
 * 模块控制目标：1、缩小x方向误差，2、缩小y方向误差，3、缩小航向角psi方向误差，4、缩小俯仰角theta方向误差
 * 模块输入：1、控制目标，2、当前状态
 * 控制目标具体量：1、x_r，2、y_r 3、psi_r = 0 4、theta_r = 0
 * 当前状态具体量：1、x，dx，2、y，dy，3、psi，dpsi，
 * 模块输出：驱动轮驱动力矩：Tal1、Tal2；
 */

 /* 首先尝试使用MPC速度控制，
  * 整个控制模块只需要使用MPControlAlgorithms模块即可，如需加上SlidingModeControlAlgorithms，
  * 则需要mpc输出速度目标，滑模控制输出力矩目标
  * 抽象的算法类实现为最终的调用接口，或者被main函数调用，或者被某个线程调用，因此，不依赖于任何数据，没有输入
  * 与抽象算法类对应的具体实现类则需要决定具体的输入来自于哪个传感器，具体输出到哪个电机
  */

  // 控制目标结构体
struct ControlTarget
{
	double x_r;
	double y_r;
	double psi_r;
};

// 状态变量结构体
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
	// MPC输入
	double exc;
	double eyc;
	double epsi;
	double vr;
};

struct NavControlOutput
{
	//控制输出
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
		//控制目标输入
		/*d_control_target.x_r = input[0];
		d_control_target.y_r = input[1];
		d_control_target.psi_r = input[2];*/

		//状态变量检测
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
		//MPC输出
		vector<double> MPCoutput;
		double ev = MPCoutput.at(0);
		double ew = MPCoutput.at(1);
		//滑模控制输入

		//滑模控制输出

		/*MPControl(exc, eyc, epsi, vr);*/
		/*MPControl.MPControlOutput(MPCoutput);*/
	}


	void ControlOutput()
	{
		/*MPControl.MPControlOutput();*/
	}
};

