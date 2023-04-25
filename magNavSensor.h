#ifndef MAGNAVSENSOR_H
#define MAGNAVSENSOR_H

#pragma once
#include "SensorInterface.h"

class magNavSensor
{
private:
	modbus_t* ctx;
	int rc; //��ȡ��Ԫ����
	int d_server_id; //�����id��
	int d_target_addr; //�������ʼ��ַ
	int16_t* distance_registers = new int16_t[3]; //
	string COMName;
	unsigned int d_MSRnum; //��⵽�Ĵ�����
	vector<int8_t> d_distance; //��������ŵ��봫�������ĵľ���
public:
	/* ���캯������RTU��������
	 * �������ʱ������:
	 * 1��COM�˿ڡ�
	 * 2�������id�š���������ڴ�������ʱȷ��
	 * 3��Ŀ���ַ�����治ͬ����ϵͳ������仯���ڲ�ͬ����ϵͳ���뵱��ȷ��
	 */
	magNavSensor(string& COM, int server_id)
	{
		d_server_id = server_id;
		COMName = COM;
		ctx = modbus_new_rtu(COMName.c_str(), 115200, 'N', 8, 1);
		modbus_set_slave(ctx, d_server_id);
		modbus_set_debug(ctx, FALSE);

	}

	~magNavSensor()
	{
		delete distance_registers;

		modbus_free(ctx);
		cout << "end magNavSensor" << endl;
	};

	/* �ŵ�������������ǰ���ȶ�ѭ��ϵͳ��Ҫ���ݸ�����ϵͳ����Ϣ����
	 * 1�������ĵ�ƫ�ƾ�����Ϣ
	 * 2��������·���ж����͵��ź�
	 *
	 */

	 /* �����ŵ����������ڼ�⵽һ��������ʱ�̣�
	  * �����ŵ������ߵ�ƫ�ƾ��룬
	  * ����Ӵ����������ǰ�������������ڴ������������ʱ�����ֵ�����������ڴ����������ұ�ʱ�������
	  */
	void Distance2MSR(vector<int8_t>& d_distance)
	{
		d_target_addr = 4;
		modbus_connect(ctx);
		rc = modbus_read_registers(ctx, d_target_addr, 3, (uint16_t*)distance_registers);
		modbus_close(ctx);
		if (rc != 3) {
			cout << "read magNavSensor false" << endl;
		}
		else {
			cout << "read magNavSensor succeed" << endl;
		}
		d_MSRnum = (unsigned int)(distance_registers[0] >> 8);
		d_distance[0] = (distance_registers[0] & 0xff);
		d_distance[1] = (distance_registers[1] >> 8);
		d_distance[2] = (distance_registers[1] & 0xff);
	}
};

/* ƫ�����ݼ�����
 * ƫ��������Ҫ���������ŵ�����������ͬ����ó�
 */
class YawCalculate :
	public Sensor2Control
{
private:
	int d_server_id1; //�����id��
	int d_server_id2; //�����id��
	string COMName1; // �ŵ���������1����COM��
	string COMName2; // �ŵ���������2����COM��
	vector<int8_t> frontoffsetvector;
	vector<int8_t> backoffsetvector;
	vector<int8_t>& frontoffset = frontoffsetvector;
	vector<int8_t>& backoffset = backoffsetvector;
	float d_Sensor2Sensor = 700.0; // ǰ�󴫸�����������һ�롣��λ��mm
	float d_lateral_distance; // �������ľ���Ԥ������ĺ�����룬�������ģ�ͣ���λ��mm
	float a_yaw_angle; // ����ƫ���Ƕȣ��������ģ�ͣ���λ��deg

public:
	/* ƫ�������Ĺ���������������Ե�id�źͶ˿ں�
	 */
	YawCalculate(string& COM1, string& COM2, int server_id1, int server_id2)
	{
		frontoffsetvector = vector<int8_t>(3);
		backoffsetvector = vector<int8_t>(3);
		d_server_id1 = server_id1;
		d_server_id2 = server_id2;
		COMName1 = COM1;
		COMName2 = COM2;
	}

	void NavigationControlCal()
	{
		magNavSensor* Sensor1 = new magNavSensor(COMName1, d_server_id1);
		magNavSensor* Sensor2 = new magNavSensor(COMName2, d_server_id2);
		Sensor1->Distance2MSR(frontoffset);
		Sensor2->Distance2MSR(backoffset);

		d_lateral_distance = 0.5 * ((int32_t)frontoffset[0] + (int32_t)backoffset[0]);
		cout << "distance1 is: " << (int32_t)frontoffset[0] << endl;
		cout << "distance2 is: " << (int32_t)backoffset[0] << endl;

		int max_offset = max(abs(frontoffset[0]), abs(backoffset[0]));
		if (max_offset == frontoffset[0] || (max_offset + frontoffset[0]) == 0) {
			max_offset = frontoffset[0];
		}
		else {
			max_offset = backoffset[0];
		}
		a_yaw_angle = atan2f(((float)max_offset - d_lateral_distance), d_Sensor2Sensor);

		/*cout << (int16_t)offset[0][0] << endl;
		cout << (int16_t)offset[1][0] << endl;*/

	}
	void NavigationControlInput(vector<float>& input)
	{
		NavigationControlCal();
		input[0] = d_lateral_distance;
		input[1] = a_yaw_angle;
	}


	void lateralStableControlInput()
	{

	}
};

#endif // !MAGNAVSENSOR_H


