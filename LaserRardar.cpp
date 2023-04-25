#include "LaserRardar.h"

LaserRardar::LaserRardar()
{
	receive_data = new uint8_t[500];
	d_LaserAddr_len = sizeof(LaserAddr);

	//初始化
	if (WSAStartup(MAKEWORD(2, 2), &wasData) != 0) {
		cout << "WSAStartup() error" << endl;
	}

	//创建套接字
	IPCSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //IPV4数据流TCP类型
	if (IPCSocket == INVALID_SOCKET) {
		cout << "socket() error" << endl;
	}

	memset(&LaserAddr, 0, sizeof(LaserAddr));
	LaserAddr.sin_family = AF_INET; //IPV4
	LaserAddr.sin_port = htons(Laser_port);
	LaserAddr.sin_addr.S_un.S_addr = inet_addr(Laser_ip);
	if (connect(IPCSocket, (sockaddr*)&LaserAddr, sizeof(SOCKADDR_IN)) == SOCKET_ERROR) {
		cout << "connect error" << endl;
		closesocket(IPCSocket);
	}
	else {
		cout << "connect success" << endl;
	}
}

LaserRardar::~LaserRardar()
{
	closesocket(IPCSocket);
	WSACleanup();
	delete receive_data;
}

void LaserRardar::StartReceiveLaserData()
{
	send(IPCSocket, (const char*)start_message, strlen((const char*)start_message), 0);

	for (;;) {
		cout << LaserRadarDataAt(180)->distance << endl;
	}
}

void LaserRardar::StopReceiveLaserData()
{
	send(IPCSocket, (const char*)stop_message, strlen((const char*)stop_message), 0);
}

SingalPointLaserRadarData* LaserRardar::LaserRadarDataAt(float angle)
{
	SingalPointLaserRadarData* data_at_singal_point = new SingalPointLaserRadarData;
	int index; // 数组索引
	if ((angle > 15) && (angle < 136)) {
		do
		{
			receive_data_len = recv(IPCSocket, (char*)receive_data, 910, 0);
			start_angle = receive_data[6] * 256 + receive_data[7];
		} while ((start_angle != 45));
		index = 12 + (angle - 15) / 0.2;

	}
	else if ((angle > 136) && (angle < 232))
	{
		do
		{
			receive_data_len = recv(IPCSocket, (char*)receive_data, 910, 0);
			start_angle = receive_data[6] * 256 + receive_data[7];
		} while (/*(start_angle != 45) &&*/ (start_angle != 136) /*&& (start_angle != 232)*/);
		index = 12 + (angle - 136) / 0.2;
	}
	else if ((angle > 232) && (angle < 315))
	{
		do
		{
			receive_data_len = recv(IPCSocket, (char*)receive_data, 910, 0);
			start_angle = receive_data[6] * 256 + receive_data[7];
		} while ((start_angle != 232));
		index = 12 + (angle - 232) / 0.2;
	}

	data_at_singal_point->highDis = receive_data[index + 1];
	data_at_singal_point->lowDis = receive_data[index + 2];
	data_at_singal_point->highStrength = receive_data[index + 3];
	data_at_singal_point->lowStrength = receive_data[index + 4];
	data_at_singal_point->distance = data_at_singal_point->highDis * 256 + data_at_singal_point->lowDis;
	data_at_singal_point->strength = data_at_singal_point->highStrength * 256 + data_at_singal_point->lowStrength;

	return data_at_singal_point;

	delete data_at_singal_point;
}

int LaserRardar::LaserRadarRangeDataFrom(float angleStart, float angleEnd)
{
	int distance = 0;
	int range = (angleEnd - angleStart) / 0.2;
	float angle = angleStart;
	d_data = new vector<SingalPointLaserRadarData*>(range);
	for (int i = 0; i < range; i++) {
		d_data->at(i) = LaserRadarDataAt(angle);
		angle += 0.2; //角度分辨率为0.2
		distance += (1.0 / range) * d_data->at(i)->distance;
	}

	return distance;
	delete d_data;
}



