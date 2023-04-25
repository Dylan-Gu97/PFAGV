#pragma once
#include "SensorInterface.h"

/*RFID��ȡ����϶࣬�����ȡ��д��ĵ�ַ������libmodbus���������󣬶����ַΪint����*/
struct RFIDAdress
{
	int ReadTargetAddress;  // �����ּĴ��� 0x03������
	int ReadWriteStatusAddress; //�����ּĴ��� 0x03������
	int WriteTargetAddress; // д������ּĴ��� 0x10������
	int CleanTargetAddress; // д�������ּĴ��� 0x06������

	RFIDAdress() {
		ReadTargetAddress = 0x0025;
		ReadWriteStatusAddress = 0x0020;
		WriteTargetAddress = 0x0017;
		CleanTargetAddress = 0x0000;
	}
};

/*�����ȡ���ݳ���*/
struct DataLength
{
	int ReadDataLength;
	int WriteDataLength;
	int ReadWriteStatusLength;
	DataLength() {
		ReadDataLength = 0x0003;
		WriteDataLength = 0x0002;
		ReadWriteStatusLength = 0x0001;
	}
};

struct WriteInstruct
{
	uint16_t CleanInstruct[1];
	uint16_t TargetContent[2];
	WriteInstruct() {
		CleanInstruct[0] = 0x0106;
	}
};

struct ReadStatus
{
	uint8_t reading;
	uint8_t not_reading;
	uint8_t RSSI;
	ReadStatus() {
		reading = 0x00;
		not_reading = 0x01;
	}
};

class RFIDSensor :
	public Sensor2Control
{
private:
	modbus_t* ctx;
	string COMName;
	int d_server_id;
	RFIDAdress* d_address;
	DataLength* d_data_length;
	WriteInstruct* d_write_intruct;

public:
	/* ���캯������RTU��������
	 * �������ʱ������:
	 * 1��COM�˿ڡ�2�������id�š���������ڴ�������ʱȷ��
	 * 3��Ŀ���ַ�����治ͬ����ϵͳ������仯���ڲ�ͬ����ϵͳ���뵱��ȷ��
	 *
	 */
	RFIDSensor(string& COM, int server_id)
	{
		d_server_id = server_id;
		COMName = COM;
		ctx = modbus_new_rtu(COMName.c_str(), 115200, 'N', 8, 1);
		modbus_set_slave(ctx, d_server_id);
		modbus_set_debug(ctx, TRUE);
		d_address = new RFIDAdress;
		d_data_length = new DataLength;
		d_write_intruct = new WriteInstruct;
		modbus_connect(ctx);
	}
	~RFIDSensor()
	{
		modbus_close(ctx);
		modbus_free(ctx);
		cout << "end RFIDSensor" << endl;
	};

	inline void RFIDRead(uint16_t current_id[3])
	{
		modbus_read_registers(ctx, d_address->ReadTargetAddress, d_data_length->ReadDataLength, current_id);
	}

	inline void RFIDWrite(uint16_t current_id[2])
	{
		modbus_write_registers(ctx, d_address->WriteTargetAddress, d_data_length->WriteDataLength, current_id);
	}

	inline void NavigationControlInput(vector<float>& input)
	{

	}
};

