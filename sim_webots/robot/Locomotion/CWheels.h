#pragma once

//#include <stdio.h>
#include <cmath>
#include "Config.h"

using namespace std;

class CWheels
{
public:
	explicit CWheels(float s = 0, float h = 0, float k = 0, float a = 0);
	~CWheels();
	// 状态
	float wl_v = 0 ; 	 	// 速度
	float wl_p = 0 ;		// 位置
	// 控制
	double wl_cmd_v = 0 ;  	// 控制速度
};
