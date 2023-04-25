#pragma once
#include "SensorInterface.h"
#include <WinSock2.h>
#include <format>

/* 雷达传感器采用tcp通讯，传递雷达传感器检测数据
 * 该传感器相当于tcp服务端，此处编写的工控机程序相当于tcp客户端
 * 需要向雷达传感器发送报文，之后建立连接，传递距离数据
 * 对于雷达传感器的数据处理，预设的方案是拟合出距离障碍物的直线，求解出距离障碍物的距离
 * 雷达默认分辨率是0.2°
 * 45°至136°有455个测量点
 * 136°至232°有480个测量点
 * 232°至315°有415个测量点
 * 根据数组下表索引对应角度数值
 */

struct SingalPointLaserRadarData
{
	BYTE highDis;
	BYTE lowDis;
	BYTE highStrength;
	BYTE lowStrength;
	int distance;
	int strength;
	SingalPointLaserRadarData() {};
};


class LaserRardar :
	public Sensor2Control
{
private:
	WSAData wasData;
	SOCKET IPCSocket;
	SOCKADDR_IN LaserAddr;
	int d_LaserAddr_len;

	const char* Laser_ip = "192.168.5.8";
	const u_long Laser_port = 8080;

	// 客户端启动接受和停止接受指令
	uint8_t start_message[8] = { (uint8_t)0x52, (uint8_t)0x41, (uint8_t)0x75, (uint8_t)0x74, (uint8_t)0x6F,(uint8_t)0x01, (uint8_t)0x87, (uint8_t)0x80 };
	uint8_t stop_message[8] = { (uint8_t)0x52,(uint8_t)0x41,(uint8_t)0x75,(uint8_t)0x74,(uint8_t)0x6f,(uint8_t)0x00,(uint8_t)0x46,(uint8_t)0x40 };

	uint8_t* receive_data; //客户端接收到的第一手资料
	int receive_data_len;
	int start_angle;

	vector<SingalPointLaserRadarData*>* d_data;

public:
	LaserRardar();
	~LaserRardar();
	void StartReceiveLaserData();
	void StopReceiveLaserData();
	// 得出某一个角度处的距离和强度结构体
	SingalPointLaserRadarData* LaserRadarDataAt(float angle);
	// 得出某一个角度范围内的距离均值
	int LaserRadarRangeDataFrom(float angleA, float angleB);
	void NavigationControlInput(vector<float>& input)
	{

	}
	void lateralStableControlInput()
	{

	}

};

