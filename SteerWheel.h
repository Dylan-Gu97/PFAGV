#pragma once
#include "SensorInterface.h"

/* ������������Modbusд���ַ������д�뱣�ּĴ�������(16)
 */
struct WriteMutipleResiterAddress
{
	int clean_obstacle_addr;
	int motorA_status_addr;
	int motorB_status_addr;
	int motorA_driving_dir_addr;
	int motorB_driving_dir_addr;
	int motorA_driving_speed_addr;
	int motorB_driving_speed_addr;

	WriteMutipleResiterAddress() {
		clean_obstacle_addr = 2000;
		motorA_status_addr = 2002;
		motorB_status_addr = 2003;
		motorA_driving_dir_addr = 2004;
		motorB_driving_dir_addr = 2005;
		motorA_driving_speed_addr = 2006;
		motorB_driving_speed_addr = 2007;
	}
};

/* ���Ĵ�����ַ */
struct ReadMultipleRegisterAddress
{
	int motorA_current_addr;
	int motorB_current_addr;
	int motorA_dir_addr;
	int motorB_dir_addr;
	int motorA_speed_addr;
	int motorA_angle_addr;
	int motorB_speed_addr;
	int motorB_angle_addr;
	int actator_status_addr;
	int actator_input_voltage_addr;
	int actator_temperature_addr;

	ReadMultipleRegisterAddress() {
		motorA_current_addr = 1000;
		motorB_current_addr = 1001;
		motorA_dir_addr = 1002;
		motorB_dir_addr = 1003;
		motorA_speed_addr = 1004;
		motorB_speed_addr = 1005;
		motorA_angle_addr = 1004;
		motorB_angle_addr = 1005;
		actator_status_addr = 1006;
		actator_input_voltage_addr = 1007;
		actator_temperature_addr = 1032;
	}
};


/* �����趨���״̬�ṹ�����,Ϊд�����
 * ͨ���趨���״̬����������ֵ���廯Ϊ�����ı�������
 */
struct MotorStatusSetRule
{
	uint16_t Disable;
	uint16_t Enable;
	uint16_t Reducer;
	uint16_t Brake;
	uint16_t OpenValve;
	uint16_t PositiveDir;
	uint16_t NegativeDir;

	MotorStatusSetRule() {
		Disable = 0;
		Enable = 1;
		Reducer = 2;
		Brake = 3;
		OpenValve = 5;
		PositiveDir = 0;
		NegativeDir = 1;
	}
};

/*
 * �����ȡ���������
 */
struct MotorErrorRule
{
	uint16_t normal;
	MotorErrorRule() {
		normal = 0;
	}
};

/* ������״̬����ṹ��
 */
struct MotorStatusDetect
{
	uint16_t currentA;
	uint16_t currentB;
	uint16_t MotorADir;
	uint16_t MotorBDir;
	uint16_t MotroASpeed;
	uint16_t MotorBSpeed;
	uint16_t actatorTemperature;
};


/*�����趨Ϊǰ�򣬱�־λΪ0��
* �����趨Ϊ���򣬱�־λ�趨Ϊ1
* �����趨Ϊ�������¿�˳ʱ��ԭ���Դ�����־λ�趨Ϊ2
* �����趨Ϊ����������ʱ��ԭ���Դ�����־λ�趨Ϊ3
*/
struct VehicleDirDef
{
	int Forward;
	int Backward;
	int Clockwise;
	int AntiClockwise;
	VehicleDirDef() {
		Forward = 0;
		Backward = 1;
		Clockwise = 2;
		AntiClockwise = 3;
	}
};

class SteerWheel :
	public Control2Actuator
{
private:
	modbus_t* ctx;
	int d_server_id; //������id���ݶ�Ϊ1�����ù�������ʱ���ⲿ����
	string COMName;

	/* ��д�ĵ�ַ�Ͷ�д�Ĺ��򣬲��ýṹ���ʾ */
	WriteMutipleResiterAddress* d_write_register_address;
	MotorStatusSetRule* d_motor_status_set_rule;
	ReadMultipleRegisterAddress* d_read_register_address;
	MotorStatusDetect* d_motor_status_detect;
	VehicleDirDef* d_vehicle_dir_set;
public:
	/* ���캯������RTU��������
	 * �������ʱ������:
	 * 1��COM�˿ڡ�2�������id�š���������ڴ�������ʱȷ��
	 * 3��Ŀ���ַ�����治ͬ����ϵͳ������仯���ڲ�ͬ����ϵͳ���뵱��ȷ��
	 */
	SteerWheel(string& COM, int server_id)
	{
		d_server_id = server_id;
		COMName = COM;
		ctx = modbus_new_rtu(COMName.c_str(), 115200, 'N', 8, 1);
		modbus_set_slave(ctx, d_server_id);
		modbus_set_debug(ctx, TRUE);

		d_write_register_address = new WriteMutipleResiterAddress;
		d_motor_status_set_rule = new MotorStatusSetRule;;
		d_read_register_address = new ReadMultipleRegisterAddress;
		d_motor_status_detect = new MotorStatusDetect;;
		d_vehicle_dir_set = new VehicleDirDef;
	}
	~SteerWheel()
	{
		delete d_write_register_address;
		delete d_motor_status_set_rule;
		delete d_read_register_address;
		delete d_motor_status_detect;
		delete d_vehicle_dir_set;

		modbus_free(ctx);
		cout << "end SteerWheel" << endl;
	}

	/* ������Ϻ�������ÿ�ε����ʱ������Ҫ���������������Ҫ�������
	 * Modbus���ܣ�д�뱣�ּĴ���(16)���Ĵ�����ַ��2000����λ������01->00
	 */
	inline void cleanObstacle()
	{
		modbus_write_register(ctx, d_write_register_address->clean_obstacle_addr, 1);
		modbus_write_register(ctx, d_write_register_address->clean_obstacle_addr, 0);
	}

	/* ���ʹ�ܺ���������A�����B�����ǰ������Ӻ���ǰ�������ΪA������Ҳ�ΪB�����*/
	inline void wheelEnable()
	{
		modbus_write_register(ctx, d_write_register_address->motorA_status_addr, d_motor_status_set_rule->Enable);
		modbus_write_register(ctx, d_write_register_address->motorB_status_addr, d_motor_status_set_rule->Enable);
	}

	/* �����բ�򿪺��� */
	inline void wheelFree()
	{
		modbus_write_register(ctx, d_write_register_address->motorA_status_addr, d_motor_status_set_rule->OpenValve);
		modbus_write_register(ctx, d_write_register_address->motorB_status_addr, d_motor_status_set_rule->OpenValve);
	}

	/* �������ֹͣ���� */
	inline void wheelReducer()
	{
		modbus_connect(ctx);
		modbus_write_register(ctx, d_write_register_address->motorA_status_addr, d_motor_status_set_rule->Reducer);
		modbus_write_register(ctx, d_write_register_address->motorB_status_addr, d_motor_status_set_rule->Reducer);
		modbus_close(ctx);
	}

	/* �����ͣ���� */
	inline void EmergencyStop()
	{
		modbus_write_register(ctx, d_write_register_address->motorA_status_addr, d_motor_status_set_rule->Brake);
		modbus_write_register(ctx, d_write_register_address->motorB_status_addr, d_motor_status_set_rule->Brake);
	}

	/* ����ٶ��趨����
	 */
	inline void MotoSpeedSet(uint16_t speedA, uint16_t speedB)
	{
		modbus_write_register(ctx, d_write_register_address->motorA_driving_speed_addr, speedA);
		modbus_write_register(ctx, d_write_register_address->motorB_driving_speed_addr, speedB);
	}

	/* �����趨����*/
	inline void VehicleDirSet(int flag)
	{
		if (flag == d_vehicle_dir_set->Forward) {
			modbus_write_register(ctx, d_write_register_address->motorA_driving_dir_addr, d_motor_status_set_rule->PositiveDir);
			modbus_write_register(ctx, d_write_register_address->motorB_driving_dir_addr, d_motor_status_set_rule->NegativeDir);
		}
		if (flag == d_vehicle_dir_set->Backward) {
			modbus_write_register(ctx, d_write_register_address->motorA_driving_dir_addr, d_motor_status_set_rule->NegativeDir);
			modbus_write_register(ctx, d_write_register_address->motorB_driving_dir_addr, d_motor_status_set_rule->PositiveDir);
		}
		if (flag == d_vehicle_dir_set->Clockwise) {
			modbus_write_register(ctx, d_write_register_address->motorA_driving_dir_addr, d_motor_status_set_rule->PositiveDir);
			modbus_write_register(ctx, d_write_register_address->motorB_driving_dir_addr, d_motor_status_set_rule->PositiveDir);
		}
		if (flag == d_vehicle_dir_set->AntiClockwise) {
			modbus_write_register(ctx, d_write_register_address->motorA_driving_dir_addr, d_motor_status_set_rule->NegativeDir);
			modbus_write_register(ctx, d_write_register_address->motorB_driving_dir_addr, d_motor_status_set_rule->NegativeDir);
		}
	}

	void NavigationControlOutput(vector<float>& output)
	{
		modbus_connect(ctx);
		wheelEnable();
		/*wheelFree();*/
		/*wheelFree();*/
		/*�����趨Ϊǰ�򣬱�־λΪ0��
			* �����趨Ϊ���򣬱�־λ�趨Ϊ1
			* �����趨Ϊ�������¿�˳ʱ��ԭ���Դ�����־λ�趨Ϊ2
			* �����趨Ϊ����������ʱ��ԭ���Դ�����־λ�趨Ϊ3
			*/
		VehicleDirSet(output[2]);
		uint16_t speedA, speedB;
		speedA = static_cast<uint16_t>(output[0] + 0.5);
		speedB = static_cast<uint16_t>(output[1] + 0.5);
		MotoSpeedSet(speedA, speedB);
		cleanObstacle();
		modbus_close(ctx);
	}

	void lateralStableControlOutput()
	{

	}
};

