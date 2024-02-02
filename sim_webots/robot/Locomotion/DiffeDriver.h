#pragma once
#include "CState.h"
#include "CWheels.h"
#include "Config.h"
#include <cmath>


class DiffeDriver
{
public:
	explicit DiffeDriver(CState& st);
	~DiffeDriver();

    IRobot *rbt = nullptr;  // 机器人接口
    CState& state;  // 机器人状态数据
    CWheels& lL;
    CWheels& lR;

    //------ 机器人移动任务管理 ---------------------------------
    LOCO_ID taskId = STANDBY;
    LOCO_ID tempId = STANDBY;
    bool    isNewId = true;
    void    task();
private:
};

