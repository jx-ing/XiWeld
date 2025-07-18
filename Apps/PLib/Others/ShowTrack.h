/*********************************************************************

* �ļ����ƣ� ShowTrack.h
* ����ժҪ��
* ��ǰ�汾�� 2.0
* ��    �ߣ� ������
* ������ڣ� 2020��03��09��

ʹ��˵����
1.�����úñ�������ɲ��裩��
2.���ù켣��ʾ������ͱ�����
3.��DXFͼ���㡢�켣��ͼ����
4.ע�⣺DrawBackgroundImage�������BitBlt�����ɼ�����˸���⣻

����˵����
1.����ͼƬ��DXF�켣�ķŴ���С������
2.�޸��˲��ֺ���ʵ�֣����ٲ���Ҫ�Ŀ��ļ�������
**********************************************************************/
#pragma once

#include "CommonAlgorithm.h"
#include "XiPlaneTypes.h"
#include "GeometryFeature.h"

typedef struct
{
	CString strLegend;
	COLORREF rgb;
}T_LEGEND;

class CShowTrack
{
public:
	CShowTrack();
	~CShowTrack();
	//����ͼƬ��ʾ������
	void SetImageWidthAndHeight(CRect rectShowWholeImage);
	//��scaleΪ�����Ŵ����СͼƬ������ͼƬ�Ⱦ�ͼƬ������tCenterΪ���Ĳü���ͼƬ��ԭͼƬ��С
	//tCenter�����ͼƬ�ķŴ����ģ������ǽ����
	//�ú�����Ҫ��ʹ��SetImageWidthAndHeight����
	//�ú������ܻ����tCenter��ֵ������ʹ����tCenter�йصĲ���ʱ����Ҫ�����µ�tCenter���¼���
	void ResizeImage(IplImage* img, IplImage* &dst, double dRatio, T_POINT_2D &tCenter);
	//��ȡ��һ�ηŴ󣨲�������С��ʱ����ʾ��ͼƬ��ԭͼ�е�����
	void GetImageROI(CRect &cRect);
	void SetImageROI(CvRect &cRect);
	//����ͼƬ����Ϊ��������Ҫ��ʹ��SetImageWidthAndHeight����
	void DrawBackgroundImage(IplImage *pBackgroundImage, CDialog *pDialog, int nStaticId);	
	//����ͼƬ����Ϊ����������Ҫ�����������
	void DrawBackgroundImage(ATL::CImage *pBackgroundImage, CDialog *pDialog, CRect &cRect);
	//���ù켣��ʾ������
	void SetDrawRegionGlobal(CRect rect);
	//��ȡ�켣��ʾ�ı��������ɿ���
	void GetDrawGlobalRatio(std::vector <T_POINT_3D> vtTrack, double &dRatio);
	//���ù켣��ʾ�ı�����ֻ������һ�Σ�
	void SetDrawGlobalRatio(double dDrawGlobalRatio);
	//tCenter����Խ���ķŴ����ģ�������ͼƬ��
	//�˺�����Ҫ���ResizeImage����ʹ�ã���Ҫע�����tCenterֵ��Ҫ����ResizeImage��tCenter���¼���
	void ResizeTrack(double dRatio, T_POINT_2D &tCenter);
	//��DXFͼֽ
	void DrawDXF(std::vector <T_PART_GRAPH> vtPartGlobalAll, CDialog *pDialog);
	//��һ���㣬dRatio�ǵ�İ뾶
	void DrawPoint(T_POINT_3D tPoint, CDialog *pDialog, COLORREF crColor = RGB(255, 215, 0), int dRatio = 6);
	//��һ�ι켣����β��������
	void DrawTrack(std::vector <T_POINT_3D> vtTrack, CDialog *pDialog, COLORREF crColor = RGB(220, 20, 60), int nLineType = 0);
	//��ͼ��
	void DrawLegends(std::vector <T_LEGEND> vtLegends, int xDest, int yDest, CDialog *pDialog);
	
private:
	void DrawGraph(std::vector <T_DXF_ENTITY_PARA> &vtGraph, CDC *pDC, double
		dMinRealX, double dMinRealY, double dDrawRatio, int nDrawOriginX, int nDrawOriginY, int nLineType = 0);
	CRect m_rectShowWholeImage;
	double m_dDrawGlobalRatio; // ������ʾ����/����̨���㷶Χ
	int m_nDrawGlobalOriginX;
	int m_nDrawGlobalOriginY;
	double m_dMinRealX;
	double m_dMinRealY;
	CRect m_rectDrawRegionGlobal;
	CRect m_rectROI;
	
};

