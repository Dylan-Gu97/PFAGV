#include "PipeConnect.h"

PipeConnect::PipeConnect()
{
	message = new MessageStruct;
	MESSAGE_SIZE = sizeof(*message);
	pipe_handle = CreateNamedPipeA(
		PIPE_NAME, PIPE_ACCESS_DUPLEX, PIPE_TYPE_BYTE | PIPE_WAIT,
		2, MESSAGE_SIZE, MESSAGE_SIZE, 0, NULL);
	if (pipe_handle == INVALID_HANDLE_VALUE) {
		std::cerr << "Error creating named pipe: " << GetLastError() << std::endl;
	}
	// 等待客户端连接
	if (ConnectNamedPipe(pipe_handle, NULL) == 0) {
		std::cerr << "Error connecting to client: " << GetLastError() << std::endl;
		delete message;
		CloseHandle(pipe_handle);
	}
	std::cout << "Client connected." << std::endl;
}

PipeConnect::~PipeConnect()
{
	delete message;
	FlushFileBuffers(pipe_handle);
	DisconnectNamedPipe(pipe_handle);
	CloseHandle(pipe_handle);
}

void PipeConnect::pipeSendMessage()
{
	if (WriteFile(pipe_handle, message, byte_read, NULL, NULL) == 0) {
		std::cerr << "Error writing to pipe: " << GetLastError() << std::endl;
	}
}

void PipeConnect::pipeReciveMessage()
{
	if (ReadFile(pipe_handle, message, MESSAGE_SIZE, &byte_read, NULL) == 0) {
		std::cerr << "Error writing to pipe: " << GetLastError() << std::endl;
	}
}
