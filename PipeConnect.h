#pragma once
#include <windows.h>
#include <iostream>

struct MessageStruct {
	bool LiftMotorStart;
	int LiftMotorSpeed[2];
};

class PipeConnect
{
private:
	int MESSAGE_SIZE;
	const char* PIPE_NAME;
	HANDLE pipe_handle;
	MessageStruct* message;
	DWORD byte_read;

public:
	PipeConnect();
	~PipeConnect();

	void pipeSendMessage();
	void pipeReciveMessage();
};

