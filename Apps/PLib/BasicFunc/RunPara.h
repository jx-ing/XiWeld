#pragma once

//读取平缝测量参数
#define PARA_FLAT_MEASURE(para) (RunPara::GetInstance().m_tFlatMeasure.para)
//读取平缝跟踪参数
#define PARA_FLAT_TRACK(para) (RunPara::GetInstance().m_tFlatTrack.para)
//读取平缝焊接参数
#define PARA_FLAT_WELD(para) (RunPara::GetInstance().m_tFlatWeld.para)
//读取立缝测量参数
#define PARA_VER_MEASURE(para) (RunPara::GetInstance().m_tVerMeasure.para)
//读取立缝焊接参数
#define PARA_VER_WELD(para) (RunPara::GetInstance().m_tVerWeld.para)

//具体参数的宏定义
#pragma region
//定义平缝测量参数
#define MACRO_FLAT_MEASURE_RUN_PARA\
    X_MACRO("dPointCloudLengthForFindEndpnt", double, dPointCloudLengthForFindEndpnt)\

//定义平缝跟踪参数
#define MACRO_FLAT_TRACK_RUN_PARA\
    X_MACRO("nSameEndPtnNumThreshold", int, nSameEndPtnNumThreshold)\

//定义平缝焊接参数
#define MACRO_FLAT_WELD_RUN_PARA\


//定义立缝测量参数
#define MACRO_VER_MEASURE_RUN_PARA\


//定义立缝焊接参数
#define MACRO_VER_WELD_RUN_PARA\


#pragma endregion

class RunPara
{
//具体参数的结构体定义
#pragma region
	struct FlatMeasure
	{
#define X_MACRO(name, type, var) type var;
		MACRO_FLAT_MEASURE_RUN_PARA
#undef X_MACRO
	};

	struct FlatTrack
	{
#define X_MACRO(name, type, var) type var;
		MACRO_FLAT_TRACK_RUN_PARA
#undef X_MACRO
	};

	struct FlatWeld
	{
#define X_MACRO(name, type, var) type var;
		MACRO_FLAT_WELD_RUN_PARA
#undef X_MACRO
	};

	struct VerMeasure
	{
#define X_MACRO(name, type, var) type var;
		MACRO_VER_MEASURE_RUN_PARA
#undef X_MACRO
	};

	struct VerWeld
	{
#define X_MACRO(name, type, var) type var;
		MACRO_VER_WELD_RUN_PARA
#undef X_MACRO
	};
#pragma endregion

private:
	RunPara();
	~RunPara();
	RunPara(const RunPara&) = delete;
	RunPara& operator=(const RunPara&) = delete;

	static RunPara m_instance;

public:
	static RunPara& GetInstance() { return m_instance; }
	bool load();

public:
	FlatMeasure m_tFlatMeasure;
	FlatTrack m_tFlatTrack;
	FlatWeld m_tFlatWeld;
	VerMeasure m_tVerMeasure;
	VerWeld m_tVerWeld;

};

