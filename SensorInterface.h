#ifndef SENSORINTERFACE
#define SENSORINTERFACE

#pragma once
#include "modbus.h"
#include "PipeConnect.h"
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
using namespace std;
#ifndef ON
#define ON 0xFF00
#define OFF 0x0000
#endif // !ON

/*
 * 传感器与执行机构输入输出接口，使用时采用vector<Sensor2Control*>类型存入不同传感器对象
 * 采用for循环执行传感器输入信号
 */

class Sensor2Control
{
public:
	virtual void NavigationControlInput(vector<float>& input) = 0;
	virtual void lateralStableControlInput() = 0;
	virtual ~Sensor2Control() { std::cout << "end sensor2Control" << std::endl; };
};

class Control2Actuator
{
public:
	virtual void NavigationControlOutput(vector<float>& output) = 0;
	virtual void lateralStableControlOutput() = 0;
	virtual ~Control2Actuator() { std::cout << "end control2Actuator" << std::endl; };
};


//class WarningData
//{
//
//};
//
//class stateData
//{
//
//};

#endif // !SENSORINTERFACE

