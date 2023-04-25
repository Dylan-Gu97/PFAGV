#pragma once
#include <iostream>
#include <mutex>
#include <Windows.h>
#include <TCHAR.H>

using namespace std;

//����log����
enum LOGLEVEL
{
	LOG_LEVEL_NONE,
	LOG_LEVEL_ERROR,     // error
	LOG_LEVEL_WARNING,   // warning
	LOG_LEVEL_DEBUG,     // debug
	LOG_LEVEL_INFO,      // info  
};

//������־���Ŀ��
enum LOGTARGET
{
	LOG_TARGET_NONE = 0x00,
	LOG_TARGET_CONSOLE = 0x01,
	LOG_TARGET_FILE = 0x10
};

class LOG
{
public:

	// ��ʼ��
	void init(LOGLEVEL loglevel, LOGTARGET logtarget);

	// 
	void uninit();

	// file
	int createFile();

	static LOG* getInstance();

	// Log����
	LOGLEVEL getLogLevel();
	void setLogLevel(LOGLEVEL loglevel);

	// Log���λ��
	LOGTARGET getLogTarget();
	void setLogTarget(LOGTARGET logtarget);

	// ��log
	static int writeLog(
		LOGLEVEL loglevel,         // Log����
		unsigned char* fileName,   // ���������ļ���
		unsigned char* function,   // ������
		int lineNumber,            // �к�
		char* format,              // ��ʽ��
		...);                      // ����

	// ���log
	static void outputToTarget();

private:
	LOG();
	~LOG();
	static LOG* Log;

	// ������
	static mutex log_mutex;

	// �洢log��buffer
	static string logBuffer;

	// Log����
	LOGLEVEL logLevel;

	// Log���λ��
	LOGTARGET logTarget;

	// Handle
	static HANDLE mFileHandle;
};


