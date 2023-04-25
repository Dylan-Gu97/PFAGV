#include "UiProcess.h"


UiProcess::UiProcess()
{
	cWinDir = new TCHAR[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, cWinDir);

	sConLin = wcscat(cWinDir, L"\\x64\\Debug\\PFAGVUI.exe");
	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	data = new UiControlData;

	CreateShareMapping();

	if (CreateProcess(sConLin, NULL, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi)) {
		cout << "create process succeed" << endl;
	}
	else {
		cerr << "failed to create process" << endl;
	};

	WaitForSingleObject(pi.hProcess, INFINITE);
}

UiProcess::~UiProcess()
{
	delete data;
	UnmapViewOfFile(lpdata);//���ӳ��
	CloseHandle(hmap);

	system("pause");
}

void UiProcess::CreateShareMapping()
{
	if (lpdata != NULL) {
		cerr << "Shared memory already exit!" << endl;
	}

	//����һ�������ֱ�ʶ�Ĺ����ڴ档
	hmap = CreateFileMappingA(INVALID_HANDLE_VALUE,
		NULL,
		PAGE_READWRITE | SEC_COMMIT,
		0,
		FILESIZE,
		FileMapping_NAME);

	if (hmap == NULL)  //������ָ��NULL����ʾ����ʧ��
	{
		cerr << "Create shared memory failed" << endl;
	}
	else {
		//ӳ���ļ���ָ��
		lpdata = MapViewOfFile(hmap, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);
		if (lpdata == NULL)  //ӳ��ʧ��
		{
			cerr << "Mapping failed!" << endl;
		}
		else
		{
			data->MotorStart = 1;
			memcpy(lpdata, data, sizeof data);  //������ڴ���д������
		}
	}
}
