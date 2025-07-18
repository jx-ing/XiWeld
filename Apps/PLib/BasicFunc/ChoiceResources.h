#pragma once

//0:无机器人 1:安川 2:埃斯顿
#define ROBOT_TYPE			1

//使能RFID读卡器功能
#define ENABLE_RFID			0

//使能梅卡曼德相机功能
#define ENABLE_MECH_EYE		0

//使能数据库功能
#define ENABLE_MY_SQL		0

//使能数据库功能
#if ENABLE_MY_SQL
#define ENABLE_UNIT_DEBUG	1
#endif

























#if ROBOT_TYPE == 1
#define ENABLE_YASKAWA_ROBOT
#elif ROBOT_TYPE == 2
#define ENABLE_ESTUN_ROBOT
#endif