#pragma once
/*
			程序统一状态返回标志，0x00000000 - 0xFFFFFFFF
                                                                          
			0xF F F F FFFF





*/
typedef enum
{
	T_STATE_FLAG_SUCCEED = 0x00000000,					//成功
	T_STATE_FLAG_COMPLATE = 0x00000001,					//完成
	T_STATE_FLAG_MANUAL_CANCEL = 0x00000002,			//手动取消




}T_STATE_FLAG;