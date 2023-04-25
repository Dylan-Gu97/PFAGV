#include "log.h"

LOG* LOG::Log = NULL;
string           LOG::logBuffer = "";
HANDLE           LOG::mFileHandle = INVALID_HANDLE_VALUE;
mutex            LOG::log_mutex;

LOG::LOG()
{
	// ��ʼ��
	init(LOG_LEVEL_NONE, LOG_TARGET_FILE);
}

void LOG::init(LOGLEVEL loglevel, LOGTARGET logtarget)
{
	setLogLevel(loglevel);
	setLogTarget(logtarget);
	createFile();
}

void LOG::uninit()
{
	if (INVALID_HANDLE_VALUE != mFileHandle)
	{
		CloseHandle(mFileHandle);
	}
}

LOG* LOG::getInstance()
{
	if (NULL == Log)
	{
		log_mutex.lock();
		if (NULL == Log)
		{
			Log = new LOG();
		}
		log_mutex.unlock();
	}
	return Log;
}

LOGLEVEL LOG::getLogLevel()
{
	return this->logLevel;
}

void LOG::setLogLevel(LOGLEVEL iLogLevel)
{
	this->logLevel = iLogLevel;
}

LOGTARGET LOG::getLogTarget()
{
	return this->logTarget;
}

void LOG::setLogTarget(LOGTARGET iLogTarget)
{
	this->logTarget = iLogTarget;
}

int LOG::createFile()
{
	TCHAR fileDirectory[256];
	GetCurrentDirectory(256, fileDirectory);

	// ����log�ļ���·��
	TCHAR logFileDirectory[256];
	_stprintf_s(logFileDirectory, _T("%s\\Test\\"), fileDirectory);// ʹ��_stprintf_s��Ҫ����ͷ�ļ�<TCHAR.H>

	// �ļ��в������򴴽��ļ���
	if (_taccess(logFileDirectory, 0) == -1)
	{
		_tmkdir(logFileDirectory);
	}

	TCHAR cTmpPath[MAX_PATH] = { 0 };
	TCHAR* lpPos = NULL;
	TCHAR cTmp = _T('\0');

	WCHAR pszLogFileName[256];
	// wcscat:�����ַ���
	wcscat(logFileDirectory, _T("test.log"));
	_stprintf_s(pszLogFileName, _T("%s"), logFileDirectory);
	mFileHandle = CreateFile(
		pszLogFileName,
		GENERIC_READ | GENERIC_WRITE,
		FILE_SHARE_READ,
		NULL,
		OPEN_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	if (INVALID_HANDLE_VALUE == mFileHandle)
	{
		return -1;
	}
	return 0;
}



void LOG::outputToTarget()
{
	if (LOG::getInstance()->getLogTarget() & LOG_TARGET_FILE)
	{
		SetFilePointer(mFileHandle, 0, NULL, FILE_END);
		DWORD dwBytesWritten = 0;
		WriteFile(mFileHandle, logBuffer.c_str(), logBuffer.length(), &dwBytesWritten, NULL);
		FlushFileBuffers(mFileHandle);
	}
	if (LOG::getInstance()->getLogTarget() & LOG_TARGET_CONSOLE)
	{
		printf("%s", logBuffer.c_str());
	}

	// ���buffer
	logBuffer.clear();
}

int LOG::writeLog(
	LOGLEVEL loglevel,         // Log����
	unsigned char* fileName,   // ���������ļ���
	unsigned char* function,   // ������
	int lineNumber,            // �к�
	char* format,              // ��ʽ��
	...)
{
	int ret = 0;

	// ��ȡ���ں�ʱ��
	SYSTEMTIME ST1
		GetSystemTime(&ST1);
	logBuffer += string(&ST1);

	// LOG����
	const char* logLevel;
	if (loglevel == LOG_LEVEL_DEBUG) {
		logLevel = "DEBUG";
	}
	else if (loglevel == LOG_LEVEL_INFO) {
		logLevel = "INFO";
	}
	else if (loglevel == LOG_LEVEL_WARNING) {
		logLevel = "WARNING";
	}
	else if (loglevel == LOG_LEVEL_ERROR) {
		logLevel = "ERROR";
	}

	// [���̺�][�̺߳�][Log����][�ļ���][������:�к�]
	char locInfo[100];
	const char* format2 = "[PID:%4d][TID:%4d][%s][%-s][%s:%4d]";
	ret = printfToBuffer(locInfo, 100, format2,
		GetCurrentProcessId(),
		GetCurrentThreadId(),
		logLevel,
		fileName,
		function,
		lineNumber);
	logBuffer += string(locInfo);

	// ��־����
	char logInfo2[256];
	va_list ap;
	va_start(ap, format);
	ret = vsnprintf(logInfo2, 256, format, ap);
	va_end(ap);

	logBuffer += string(logInfo2);
	logBuffer += string("\n");

	outputToTarget();

	return 0;
}