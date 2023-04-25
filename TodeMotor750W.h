#pragma once
#include "modbus.h"
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
using namespace std;

/* 拓达伺服电机驱动器TSDA-C21B写单个寄存器数据Modbus地址
 * 详见驱动器使用手册30页
 */
struct MotorInstructAddress
{
	int start_or_stop_address;
	int control_mode_address;
	int objective_current_address;
	int objective_speed_address;
	int accelerate_time_in_position_mode_address;
	int accelerate_time_in_speed_mode_address;
	int clean_obstacle_addr;
	int fast_stop_instruction_address;
	int slow_stop_instrction_address;
	MotorInstructAddress() {
		start_or_stop_address = 0x00;
		control_mode_address = 0x02;
		objective_current_address = 0x08;
		objective_speed_address = 0x06;
		accelerate_time_in_position_mode_address = 0x09;
		accelerate_time_in_speed_mode_address = 0x0a;
		clean_obstacle_addr = 0x4a;
		fast_stop_instruction_address = 0x4d;
		slow_stop_instrction_address = 0x4f;
	}
};

struct ControlInstruct
{
	uint16_t stop;
	uint16_t start;
	uint16_t speed_mode;
	uint16_t position_mode;
	uint16_t torque_mode;
	uint16_t fast_stop;
	uint16_t slow_stop;
	ControlInstruct() {
		stop = 0x00;
		start = 0x01;
		speed_mode = 0xc4;
		position_mode = 0xd0;
		torque_mode = 0xc1;
		fast_stop = 0x00;
		slow_stop = 0x00;
	}
};

/*拓达电机，实例化为一个大升降电机，一个小升降电机*/
class TodeMotor750W
{
private:
	modbus_t* ctx;
	int d_server_id; //驱动器id，暂定为5，调用构建函数时由外部输入
	string COMName;
	MotorInstructAddress* d_write_register_address;
	ControlInstruct* d_motor_status_set_rule;
public:
	TodeMotor750W(string& COM, int server_id);
	~TodeMotor750W();
	/* 清除故障函数，在每次电机打开时，如需要启动驱动电机则需要清除故障
	 * Modbus功能：写入保持寄存器(16)，寄存器地址：2000，复位操作：01->00
	 */
	inline void cleanObstacle()
	{
		modbus_write_register(ctx, d_write_register_address->clean_obstacle_addr, 1);
	}
	/* 电机使能函数*/
	inline void MotorEnable()
	{
		modbus_write_register(ctx, d_write_register_address->start_or_stop_address, d_motor_status_set_rule->start);
	}
	/* 电机使能函数，包括A电机和B电机（前进方向从后向前看：左侧为A电机，右侧为B电机）*/
	void MotorDisable()
	{
		modbus_write_register(ctx, d_write_register_address->start_or_stop_address, d_motor_status_set_rule->stop);
	}
	/* 电机模式设定函数 */
	inline void MotorModeSet(uint16_t mode)
	{
		if (mode == d_motor_status_set_rule->speed_mode) {
			modbus_write_register(ctx, d_write_register_address->control_mode_address, d_motor_status_set_rule->speed_mode);
		}
		else if (mode == d_motor_status_set_rule->position_mode) {
			modbus_write_register(ctx, d_write_register_address->control_mode_address, d_motor_status_set_rule->position_mode);
		}
		else if (mode == d_motor_status_set_rule->torque_mode) {
			modbus_write_register(ctx, d_write_register_address->control_mode_address, d_motor_status_set_rule->torque_mode);
		}
	}
	/* 电机电流设定函数 */
	void currentSet(uint16_t current)
	{
		modbus_write_register(ctx, d_write_register_address->objective_current_address, current);
	}
	//加速时间设定函数
	void accelerateTimeSet(uint16_t time)
	{
		uint16_t* mode;
		modbus_read_registers(ctx, d_write_register_address->control_mode_address, 1, mode);
		if (*mode == d_motor_status_set_rule->speed_mode) {
			modbus_write_register(ctx, d_write_register_address->accelerate_time_in_speed_mode_address, time);
		}
		if (*mode == d_motor_status_set_rule->position_mode) {
			modbus_write_register(ctx, d_write_register_address->accelerate_time_in_position_mode_address, time);
		}
	}
	/* 电机急停函数 */
	inline void EmergencyStop()
	{
		modbus_write_register(ctx, d_write_register_address->fast_stop_instruction_address, d_motor_status_set_rule->fast_stop);
	}
	/*电机缓慢停止指令*/
	inline void SafeStop()
	{
		modbus_write_register(ctx, d_write_register_address->slow_stop_instrction_address, d_motor_status_set_rule->slow_stop);
	}
	/* 电机速度设定函数 */
	inline void MotoSpeedSet(uint16_t speed)
	{
		modbus_write_register(ctx, d_write_register_address->objective_speed_address, speed);
	}
	/* 键盘启动函数 */
	void TodeMotorKeyboardStart(uint16_t speed);
	/* 键盘停止函数 */
	void TodeMotorKeyboardStop();
};


