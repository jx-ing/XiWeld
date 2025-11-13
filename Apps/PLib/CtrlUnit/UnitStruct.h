#pragma once

typedef struct
{
	//IN输入
	int nMagnetizingSuccessSignal;//充磁成功信号
	int nDemagnetizingSuccessSignal;//退磁成功信号
	int nAdsorptionFeedbackSignal1;//吸盘反馈信号1
	int nAdsorptionFeedbackSignal2;//吸盘反馈信号2
	std::vector<int> vtAdsorptionFeedbackSignal;//吸盘反馈信号
	int nCompactSignal1;//压紧信号1
	int nCompactSignal2;//压紧信号2
	std::vector<int> vtCompactSignal;//压紧信号

	//OUT输出
	int	nLockIO; //锁定		
	int	nDemagnetizionIO;//退磁信号		
	int anMagnetizingIO[3];//设置充磁强度io
	int MagnetizingSignIO; //充磁信号
	std::vector<int> vnMagnetGroupIO;//电磁铁组
}T_MAGENT_IO;//电磁铁IO

typedef struct
{
	//IN输入
	std::vector<int> vtPalletDetectionSignal;//托盘检测信号

	//OUT输出


}T_PALLAET_IO;//电磁铁IO1

typedef struct
{
	int nTemplateNo;
	double dCenterX;
	double dCenterY;
}T_Template_PARA;

static bool cmpRecognitionParaHeight_FromSmallToLarge(T_RECOGNITION_PARA t1, T_RECOGNITION_PARA t2) {
	return t1.dPartHeight < t2.dPartHeight;
}

static bool cmpRecognitionParaHeight_FromLargeToSmall(T_RECOGNITION_PARA t1, T_RECOGNITION_PARA t2) {
	return t1.dPartHeight > t2.dPartHeight;
}

typedef struct
{
	int *pnNo;
	CString strName;
	CString strDescription;
}T_NEW_IO;