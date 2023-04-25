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
 * ��������ִ�л�����������ӿڣ�ʹ��ʱ����vector<Sensor2Control*>���ʹ��벻ͬ����������
 * ����forѭ��ִ�д����������ź�
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

