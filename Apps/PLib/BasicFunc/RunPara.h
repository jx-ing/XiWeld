#pragma once

//��ȡƽ���������
#define PARA_FLAT_MEASURE(para) (RunPara::GetInstance().m_tFlatMeasure.para)
//��ȡƽ����ٲ���
#define PARA_FLAT_TRACK(para) (RunPara::GetInstance().m_tFlatTrack.para)
//��ȡƽ�캸�Ӳ���
#define PARA_FLAT_WELD(para) (RunPara::GetInstance().m_tFlatWeld.para)
//��ȡ�����������
#define PARA_VER_MEASURE(para) (RunPara::GetInstance().m_tVerMeasure.para)
//��ȡ���캸�Ӳ���
#define PARA_VER_WELD(para) (RunPara::GetInstance().m_tVerWeld.para)

//��������ĺ궨��
#pragma region
//����ƽ���������
#define MACRO_FLAT_MEASURE_RUN_PARA\
    X_MACRO("dPointCloudLengthForFindEndpnt", double, dPointCloudLengthForFindEndpnt)\

//����ƽ����ٲ���
#define MACRO_FLAT_TRACK_RUN_PARA\
    X_MACRO("nSameEndPtnNumThreshold", int, nSameEndPtnNumThreshold)\

//����ƽ�캸�Ӳ���
#define MACRO_FLAT_WELD_RUN_PARA\


//���������������
#define MACRO_VER_MEASURE_RUN_PARA\


//�������캸�Ӳ���
#define MACRO_VER_WELD_RUN_PARA\


#pragma endregion

class RunPara
{
//��������Ľṹ�嶨��
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

