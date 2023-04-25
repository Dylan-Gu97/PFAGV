#pragma once
#include <Windows.h>
#include <iostream>
#define _CRT_SECURE_NO_WARNINGS

struct UiControlData {
	bool MotorStart;
};

constexpr auto FileMapping_NAME = "ShareMapping";
constexpr auto FILESIZE = 4096;

/* 该类完成两件事情：
 * 1、创建指向ui界面的子进程
 * 2、创建与ui界面关联的共享内存区，并且维护该共享内存区
 */
using namespace std;

class UiProcess
{
private:
	STARTUPINFO si;
	PROCESS_INFORMATION pi;
	char cWindowsDirectory[MAX_PATH];
	LPTSTR cWinDir;
	LPCWSTR sConLin;

	LPVOID lpdata = NULL;//指针标识首地址
	HANDLE hmap; //共享内存区标识
	UiControlData* data;

public:
	UiProcess();
	~UiProcess();
	void CreateShareMapping();

};

