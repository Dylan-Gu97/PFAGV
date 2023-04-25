#pragma once
#include <Windows.h>
#include <iostream>
#define _CRT_SECURE_NO_WARNINGS

struct UiControlData {
	bool MotorStart;
};

constexpr auto FileMapping_NAME = "ShareMapping";
constexpr auto FILESIZE = 4096;

/* ��������������飺
 * 1������ָ��ui������ӽ���
 * 2��������ui��������Ĺ����ڴ���������ά���ù����ڴ���
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

	LPVOID lpdata = NULL;//ָ���ʶ�׵�ַ
	HANDLE hmap; //�����ڴ�����ʶ
	UiControlData* data;

public:
	UiProcess();
	~UiProcess();
	void CreateShareMapping();

};

