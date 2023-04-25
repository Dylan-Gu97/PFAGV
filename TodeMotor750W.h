#pragma once
#include "modbus.h"
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
using namespace std;

/* �ش��ŷ����������TSDA-C21Bд�����Ĵ�������Modbus��ַ
 * ���������ʹ���ֲ�30ҳ
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

/*�ش�����ʵ����Ϊһ�������������һ��С�������*/
class TodeMotor750W
{
private:
	modbus_t* ctx;
	int d_server_id; //������id���ݶ�Ϊ5�����ù�������ʱ���ⲿ����
	string COMName;
	MotorInstructAddress* d_write_register_address;
	ControlInstruct* d_motor_status_set_rule;
public:
	TodeMotor750W(string& COM, int server_id);
	~TodeMotor750W();
	/* ������Ϻ�������ÿ�ε����ʱ������Ҫ���������������Ҫ�������
	 * Modbus���ܣ�д�뱣�ּĴ���(16)���Ĵ�����ַ��2000����λ������01->00
	 */
	inline void cleanObstacle()
	{
		modbus_write_register(ctx, d_write_register_address->clean_obstacle_addr, 1);
	}
	/* ���ʹ�ܺ���*/
	inline void MotorEnable()
	{
		modbus_write_register(ctx, d_write_register_address->start_or_stop_address, d_motor_status_set_rule->start);
	}
	/* ���ʹ�ܺ���������A�����B�����ǰ������Ӻ���ǰ�������ΪA������Ҳ�ΪB�����*/
	void MotorDisable()
	{
		modbus_write_register(ctx, d_write_register_address->start_or_stop_address, d_motor_status_set_rule->stop);
	}
	/* ���ģʽ�趨���� */
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
	/* ��������趨���� */
	void currentSet(uint16_t current)
	{
		modbus_write_register(ctx, d_write_register_address->objective_current_address, current);
	}
	//����ʱ���趨����
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
	/* �����ͣ���� */
	inline void EmergencyStop()
	{
		modbus_write_register(ctx, d_write_register_address->fast_stop_instruction_address, d_motor_status_set_rule->fast_stop);
	}
	/*�������ָֹͣ��*/
	inline void SafeStop()
	{
		modbus_write_register(ctx, d_write_register_address->slow_stop_instrction_address, d_motor_status_set_rule->slow_stop);
	}
	/* ����ٶ��趨���� */
	inline void MotoSpeedSet(uint16_t speed)
	{
		modbus_write_register(ctx, d_write_register_address->objective_speed_address, speed);
	}
	/* ������������ */
	void TodeMotorKeyboardStart(uint16_t speed);
	/* ����ֹͣ���� */
	void TodeMotorKeyboardStop();
};


