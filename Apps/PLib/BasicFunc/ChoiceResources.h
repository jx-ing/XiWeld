#pragma once

//0:�޻����� 1:���� 2:��˹��
#define ROBOT_TYPE			1

//ʹ��RFID����������
#define ENABLE_RFID			0

//ʹ��÷�������������
#define ENABLE_MECH_EYE		0

//ʹ�����ݿ⹦��
#define ENABLE_MY_SQL		0

//ʹ�����ݿ⹦��
#if ENABLE_MY_SQL
#define ENABLE_UNIT_DEBUG	1
#endif

























#if ROBOT_TYPE == 1
#define ENABLE_YASKAWA_ROBOT
#elif ROBOT_TYPE == 2
#define ENABLE_ESTUN_ROBOT
#endif