#pragma once
#include <iostream>
#include <vector>
/*
 * �㷨�ӿ�������������㷨�ľ���ʵ�������ڸýӿڶ��������ڴ��������
 * ·�������㷨�ڲ�Ӧ���������������ϣ�MPC��SlideMode
 *
 */
class AlgorithmsInterface
{
public:
	virtual void ControlInput() = 0;
	virtual void ControlOutput() = 0;
	virtual ~AlgorithmsInterface() { std::cout << "end Algorithms" << std::endl; };
};

