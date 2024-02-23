#pragma once

// ======= type defines ====================================================================
// 行走状态宏定义
enum LOCO_ID {STANDBY=0, STAND_Hz, LEAN, WALK, FALL, MC_OFF, IDLY, STOPLOOP=99};
// 控制命令宏定义
typedef struct {
    float vx;
    float vy;
    float wz;
}RC_t;

// ================== UDP 传输的IP 地址设置 ===================================
#define PORT 8868          // 0~1024一般给系统，最大到65535
#define IP "127.0.0.1"     // 本机
#define ITS  20             // Int 单位毫秒 ms;
// ================== 机器人参数 ===================================
#define RobotMode           2           // 车辆模型，1-差速 ， 2-阿克曼 ， 3-全向
#define wheel_separation    0.86*2      // 轮间距
#define wheel_radius        0.5         // 轮子半径
#define wheel_base          2.94        // 车轴距