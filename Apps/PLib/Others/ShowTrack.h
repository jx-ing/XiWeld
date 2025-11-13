/*********************************************************************

* 文件名称： ShowTrack.h
* 内容摘要：
* 当前版本： 2.0
* 作    者： 江文奇
* 完成日期： 2020年03月09日

使用说明：
1.先设置好背景（亦可不设）；
2.设置轨迹显示的区域和比例；
3.画DXF图、点、轨迹或图例；
4.注意：DrawBackgroundImage函数配合BitBlt函数可减少闪烁问题；

更新说明：
1.新增图片和DXF轨迹的放大缩小函数；
2.修改了部分函数实现，减少不必要的库文件依赖；
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
	//设置图片显示的区域
	void SetImageWidthAndHeight(CRect rectShowWholeImage);
	//以scale为比例放大或缩小图片，如新图片比旧图片大，则以tCenter为中心裁剪新图片至原图片大小
	//tCenter是相对图片的放大中心，而不是界面的
	//该函数需要先使用SetImageWidthAndHeight函数
	//该函数可能会更改tCenter的值，继续使用与tCenter有关的参数时，需要利用新的tCenter重新计算
	void ResizeImage(IplImage* img, IplImage* &dst, double dRatio, T_POINT_2D &tCenter);
	//获取上一次放大（不用于缩小）时，显示的图片在原图中的区域
	void GetImageROI(CRect &cRect);
	void SetImageROI(CvRect &cRect);
	//加载图片设其为背景，需要先使用SetImageWidthAndHeight函数
	void DrawBackgroundImage(IplImage *pBackgroundImage, CDialog *pDialog, int nStaticId);	
	//加载图片设其为背景，不需要其他函数配合
	void DrawBackgroundImage(ATL::CImage *pBackgroundImage, CDialog *pDialog, CRect &cRect);
	//设置轨迹显示的区域
	void SetDrawRegionGlobal(CRect rect);
	//获取轨迹显示的比例（不可靠）
	void GetDrawGlobalRatio(std::vector <T_POINT_3D> vtTrack, double &dRatio);
	//设置轨迹显示的比例（只需设置一次）
	void SetDrawGlobalRatio(double dDrawGlobalRatio);
	//tCenter是相对界面的放大中心，而不是图片的
	//此函数需要配合ResizeImage函数使用，需要注意的是tCenter值需要根据ResizeImage的tCenter重新计算
	void ResizeTrack(double dRatio, T_POINT_2D &tCenter);
	//画DXF图纸
	void DrawDXF(std::vector <T_PART_GRAPH> vtPartGlobalAll, CDialog *pDialog);
	//画一个点，dRatio是点的半径
	void DrawPoint(T_POINT_3D tPoint, CDialog *pDialog, COLORREF crColor = RGB(255, 215, 0), int dRatio = 6);
	//画一段轨迹（首尾不相连）
	void DrawTrack(std::vector <T_POINT_3D> vtTrack, CDialog *pDialog, COLORREF crColor = RGB(220, 20, 60), int nLineType = 0);
	//画图例
	void DrawLegends(std::vector <T_LEGEND> vtLegends, int xDest, int yDest, CDialog *pDialog);
	
private:
	void DrawGraph(std::vector <T_DXF_ENTITY_PARA> &vtGraph, CDC *pDC, double
		dMinRealX, double dMinRealY, double dDrawRatio, int nDrawOriginX, int nDrawOriginY, int nLineType = 0);
	CRect m_rectShowWholeImage;
	double m_dDrawGlobalRatio; // 界面显示区域/工作台拍摄范围
	int m_nDrawGlobalOriginX;
	int m_nDrawGlobalOriginY;
	double m_dMinRealX;
	double m_dMinRealY;
	CRect m_rectDrawRegionGlobal;
	CRect m_rectROI;
	
};

