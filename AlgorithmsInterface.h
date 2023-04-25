#pragma once
#include <iostream>
#include <vector>
/*
 * 算法接口设计意义在于算法的具体实现依赖于该接口而不依赖于传感器输出
 * 路径跟踪算法内部应当包含两个类的组合：MPC和SlideMode
 *
 */
class AlgorithmsInterface
{
public:
	virtual void ControlInput() = 0;
	virtual void ControlOutput() = 0;
	virtual ~AlgorithmsInterface() { std::cout << "end Algorithms" << std::endl; };
};

