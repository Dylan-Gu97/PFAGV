#include "TodeMotor750W.h"

TodeMotor750W::TodeMotor750W(string& COM, int server_id)
{
	d_server_id = server_id;
	COMName = COM;
	ctx = modbus_new_rtu(COMName.c_str(), 57600, 'N', 8, 1);
	modbus_set_slave(ctx, d_server_id);
	modbus_set_debug(ctx, TRUE);

	d_write_register_address = new MotorInstructAddress;
	d_motor_status_set_rule = new ControlInstruct;
}

TodeMotor750W::~TodeMotor750W()
{
	modbus_free(ctx);
	cout << "end TodeMotor" << endl;
}

void TodeMotor750W::TodeMotorKeyboardStart(uint16_t speed)
{
	modbus_connect(ctx);
	cleanObstacle();
	MotorModeSet(d_motor_status_set_rule->speed_mode);
	MotoSpeedSet(speed);
	MotorEnable();
	modbus_close(ctx);
}

void TodeMotor750W::TodeMotorKeyboardStop()
{
	modbus_connect(ctx);
	EmergencyStop();
	modbus_close(ctx);
}

