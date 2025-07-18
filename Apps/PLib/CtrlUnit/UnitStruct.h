#pragma once

typedef struct
{
	//IN����
	int nMagnetizingSuccessSignal;//��ųɹ��ź�
	int nDemagnetizingSuccessSignal;//�˴ųɹ��ź�
	int nAdsorptionFeedbackSignal1;//���̷����ź�1
	int nAdsorptionFeedbackSignal2;//���̷����ź�2
	std::vector<int> vtAdsorptionFeedbackSignal;//���̷����ź�
	int nCompactSignal1;//ѹ���ź�1
	int nCompactSignal2;//ѹ���ź�2
	std::vector<int> vtCompactSignal;//ѹ���ź�

	//OUT���
	int	nLockIO; //����		
	int	nDemagnetizionIO;//�˴��ź�		
	int anMagnetizingIO[3];//���ó��ǿ��io
	int MagnetizingSignIO; //����ź�
	std::vector<int> vnMagnetGroupIO;//�������
}T_MAGENT_IO;//�����IO

typedef struct
{
	//IN����
	std::vector<int> vtPalletDetectionSignal;//���̼���ź�

	//OUT���


}T_PALLAET_IO;//�����IO1

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