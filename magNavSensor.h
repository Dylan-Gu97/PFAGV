#ifndef MAGNAVSENSOR_H
#define MAGNAVSENSOR_H

#pragma once
#include "SensorInterface.h"

class magNavSensor
{
private:
	modbus_t* ctx;
	int rc; //读取的元素数
	int d_server_id; //服务端id号
	int d_target_addr; //服务端起始地址
	int16_t* distance_registers = new int16_t[3]; //
	string COMName;
	unsigned int d_MSRnum; //检测到的磁条数
	vector<int8_t> d_distance; //最多三条磁道与传感器中心的距离
public:
	/* 构造函数创建RTU类型容器
	 * 建造对象时，输入:
	 * 1、COM端口。
	 * 2、服务端id号。该两项可在创建对象时确定
	 * 3、目标地址。跟随不同控制系统的需求变化，在不同控制系统输入当中确定
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

	/* 磁导航传感器对于前向稳定循迹系统需要传递给控制系统的信息在于
	 * 1、与中心的偏移距离信息
	 * 2、产生岔路的中断类型的信号
	 *
	 */

	 /* 单个磁导航传感器在检测到一条磁条的时刻，
	  * 输出与磁道中心线的偏移距离，
	  * 定义从传感器后侧向前看，磁条中心在传感器中心左边时输出正值，磁条中心在传感器中心右边时输出负数
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

/* 偏航数据计算类
 * 偏航数据需要依靠两个磁导航传感器共同计算得出
 */
class YawCalculate :
	public Sensor2Control
{
private:
	int d_server_id1; //服务端id号
	int d_server_id2; //服务端id号
	string COMName1; // 磁导航传感器1所在COM口
	string COMName2; // 磁导航传感器2所在COM口
	vector<int8_t> frontoffsetvector;
	vector<int8_t> backoffsetvector;
	vector<int8_t>& frontoffset = frontoffsetvector;
	vector<int8_t>& backoffset = backoffsetvector;
	float d_Sensor2Sensor = 700.0; // 前后传感器纵向距离的一半。单位：mm
	float d_lateral_distance; // 车体中心距离预定轨道的横向距离，输入控制模型，单位：mm
	float a_yaw_angle; // 车体偏航角度，输入控制模型，单位：deg

public:
	/* 偏航测量的构建函数，输入各自的id号和端口号
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


