#pragma once
#include "SensorInterface.h"
#include <WinSock2.h>
#include <format>

/* �״ﴫ��������tcpͨѶ�������״ﴫ�����������
 * �ô������൱��tcp����ˣ��˴���д�Ĺ��ػ������൱��tcp�ͻ���
 * ��Ҫ���״ﴫ�������ͱ��ģ�֮�������ӣ����ݾ�������
 * �����״ﴫ���������ݴ���Ԥ��ķ�������ϳ������ϰ����ֱ�ߣ����������ϰ���ľ���
 * �״�Ĭ�Ϸֱ�����0.2��
 * 45����136����455��������
 * 136����232����480��������
 * 232����315����415��������
 * ���������±�������Ӧ�Ƕ���ֵ
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

	// �ͻ����������ܺ�ֹͣ����ָ��
	uint8_t start_message[8] = { (uint8_t)0x52, (uint8_t)0x41, (uint8_t)0x75, (uint8_t)0x74, (uint8_t)0x6F,(uint8_t)0x01, (uint8_t)0x87, (uint8_t)0x80 };
	uint8_t stop_message[8] = { (uint8_t)0x52,(uint8_t)0x41,(uint8_t)0x75,(uint8_t)0x74,(uint8_t)0x6f,(uint8_t)0x00,(uint8_t)0x46,(uint8_t)0x40 };

	uint8_t* receive_data; //�ͻ��˽��յ��ĵ�һ������
	int receive_data_len;
	int start_angle;

	vector<SingalPointLaserRadarData*>* d_data;

public:
	LaserRardar();
	~LaserRardar();
	void StartReceiveLaserData();
	void StopReceiveLaserData();
	// �ó�ĳһ���Ƕȴ��ľ����ǿ�Ƚṹ��
	SingalPointLaserRadarData* LaserRadarDataAt(float angle);
	// �ó�ĳһ���Ƕȷ�Χ�ڵľ����ֵ
	int LaserRadarRangeDataFrom(float angleA, float angleB);
	void NavigationControlInput(vector<float>& input)
	{

	}
	void lateralStableControlInput()
	{

	}

};

