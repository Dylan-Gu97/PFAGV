#pragma once
#include "SensorInterface.h"
#include "magNavSensor.h"
#include "SteerWheel.h"
#include "UiProcess.h"
#include "TodeMotor750W.h"
#include <vector>
#include <Windows.h>
#include <conio.h>
#include <tchar.h>

class KeyboardControl
{
private:
	string COMPort;
	vector<float> wheelcontrol;
	vector<Control2Actuator*> Control2ActuatorVector;
	SteerWheel* steer_wheel;
	ControlInstruct* todeInstruct;
	TodeMotor750W* LiftMotor;
	TodeMotor750W* smallLiftMotor;

public:
	KeyboardControl();
	~KeyboardControl();
	void KeyboardControlStart();
};

