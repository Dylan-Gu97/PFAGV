#include "KeyboardControl.h"

KeyboardControl::KeyboardControl()
{
	COMPort = "COM7";
	wheelcontrol = vector<float>(3);
	Control2ActuatorVector = vector<Control2Actuator*>(3);
	todeInstruct = new ControlInstruct();

	steer_wheel = new SteerWheel(COMPort, 1);
	Control2ActuatorVector[0] = steer_wheel;
	LiftMotor = new TodeMotor750W(COMPort, 6);
	smallLiftMotor = new TodeMotor750W(COMPort, 5);
}

KeyboardControl::~KeyboardControl()
{
	delete steer_wheel;
	delete todeInstruct;
	delete LiftMotor;
	delete smallLiftMotor;
}

void KeyboardControl::KeyboardControlStart()
{
	if (_kbhit()) {
		char c1 = _getch();
		char c2 = _getch();
		switch (c2) {
			//键盘中的↑键表示前进
		case 72:
			/*wheelcontrol = { 400, 400, 0 };
			Control2ActuatorVector[0]->NavigationControlOutput(wheelcontrol);*/
			for (int i = 100; i < 200; i++) {
				wheelcontrol = { (float)(400 - i), (float)(400 - i), 1 };
				Control2ActuatorVector[0]->NavigationControlOutput(wheelcontrol);
			}
			/*for (int i = 200; i < 10; i--) {
				wheelcontrol = { (float)i, (float)i, 0 };
				Control2ActuatorVector[0]->NavigationControlOutput(wheelcontrol);
			}*/
			break;
			//键盘中的↓键表示后退
		case 80:
			wheelcontrol = { 400, 400, 1 };
			Control2ActuatorVector[0]->NavigationControlOutput(wheelcontrol);
			break;
			//键盘中的←键表示顺时针旋转
		case 75:
			wheelcontrol = { 400, 400, 3 };
			Control2ActuatorVector[0]->NavigationControlOutput(wheelcontrol);
			break;
			//键盘中的→键表示逆时针旋转
		case 77:
			/*for (int i = 0; i < 10; i++) {

			}*/
			wheelcontrol = { 400, 400, 2 };
			Control2ActuatorVector[0]->NavigationControlOutput(wheelcontrol);
			break;
			//键盘w键表示向上升降
		case 119:
			LiftMotor->TodeMotorKeyboardStart(-400);
			break;
			//键盘s键表示向下升降
		case 115:
			LiftMotor->TodeMotorKeyboardStart(400);
			break;
		case 97:
			smallLiftMotor->TodeMotorKeyboardStart(500);
			break;
			//键盘中D键表示小升降向下升降
		case 100:
			smallLiftMotor->TodeMotorKeyboardStart(-500);
			break;
		case 32:
			LiftMotor->TodeMotorKeyboardStop();
		default:
			LiftMotor->TodeMotorKeyboardStop();
			steer_wheel->wheelReducer();
		}
	}
	else {
		steer_wheel->wheelReducer();
		/*LiftMotor->TodeMotorKeyboardStop();*/
		//smallLiftMotor->TodeMotorKeyboardStop();
	}
}
