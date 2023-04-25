#include "PFAGVMain.h"


int main()
{
	//UiProcess Ui;
	//Ui.~UiProcess();

	string COMPort1("COM7");
	string COMPort2("COM7");
	/*vector<Sensor2Control*> Sensor2ControlVector;*/
	/*vector<Control2Actuator*> Control2ActuatorVector;*/
	/*SteerWheel* po = new SteerWheel(COMPort1, 1);*/
	/*Sensor2ControlVector.push_back(new YawCalculate(COMPort1, COMPort2, 2, 3));*/
	/*Control2ActuatorVector.push_back(new SteerWheel(COMPort1, 1));*/
	KeyboardControl keyborad_control;

	LaserRardar rardar;

	for (;;) {

		keyborad_control.KeyboardControlStart();

	}

	/*rardar.StartReceiveLaserData();
	rardar.StopReceiveLaserData();*/

	return 0;
}