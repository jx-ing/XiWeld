#include "stdafx.h"
#include ".\Apps\PLib\Others\ShowTrack.h"
#include "math.h"
#include "CvvImage.h"

using namespace std;

CShowTrack::CShowTrack()
{
	m_rectROI.left = 0;
	m_rectROI.top = 0;
	m_rectROI.right = 1024;
	m_rectROI.bottom = 1024;
}


CShowTrack::~CShowTrack()
{
}

void CShowTrack::SetImageWidthAndHeight(CRect rectShowWholeImage)
{
	m_rectShowWholeImage = rectShowWholeImage;
}

void CShowTrack::DrawBackgroundImage(IplImage *pBackgroundImage, CDialog *pDialog, int nStaticId)
{
	CDC* pDC = pDialog->GetDlgItem(nStaticId)->GetDC();			// 获得显示控件的DC
	HDC hDC = pDC->GetSafeHdc();								// 获取 HDC(设备句柄)来进行绘图操作
	CvvImage cimg;
	cimg.CopyOf(pBackgroundImage);								// 复制图片
	cimg.DrawToHDC(hDC, &m_rectShowWholeImage);					// 将图片绘制到显示控件的指定区域内
	pDialog->ReleaseDC(pDC);
}

void CShowTrack::DrawBackgroundImage(ATL::CImage *pBackgroundImage, CDialog *pDialog, CRect &cRect)
{
	pBackgroundImage->Draw(pDialog->GetDC()->m_hDC, cRect);
}

void CShowTrack::DrawDXF(std::vector <T_PART_GRAPH> vtPartGlobalAll, CDialog *pDialog)
{
	//画DXF
	CDC *pDC = pDialog->GetDC();
	CPen m_cPenOuter;
	m_cPenOuter.CreatePen(PS_SOLID, 1, RGB(0, 10, 255));

	std::vector <T_PART_GRAPH> m_vtPartGlobalAll; // 存储识别结果，包括轮廓和坡口线
	std::vector <VT_GRAPH> m_vtPartGlobal; // 存储识别结果（外轮廓）
	m_vtPartGlobalAll = vtPartGlobalAll;
	m_vtPartGlobal.clear();
	int nPartNo;
	int m_nPartNum = m_vtPartGlobalAll.size();

	for (nPartNo = 0; nPartNo < m_nPartNum; nPartNo++)
	{
		m_vtPartGlobal.push_back(m_vtPartGlobalAll[nPartNo].vtDxfEntitiesOuter);
	}

	for (nPartNo = 0; nPartNo < m_nPartNum; nPartNo++)
	{
		pDC->SelectObject(m_cPenOuter);
		DrawGraph(m_vtPartGlobal[nPartNo], pDC, 0, 0, m_dDrawGlobalRatio,
			m_nDrawGlobalOriginX, m_nDrawGlobalOriginY);
	}
	for (nPartNo = 0; nPartNo < m_nPartNum; nPartNo++)
	{
		pDC->SelectObject(m_cPenOuter);

		for (int nGraphNo = 0; nGraphNo < m_vtPartGlobalAll[nPartNo].vvtDxfEntitiesInner.size(); nGraphNo++) //tPartSingle.vvtDxfEntitiesOuterUp.size()
		{
			DrawGraph(m_vtPartGlobalAll[nPartNo].vvtDxfEntitiesInner[nGraphNo], pDC, 0, 0, m_dDrawGlobalRatio,
				m_nDrawGlobalOriginX, m_nDrawGlobalOriginY);
		}
	}
	m_cPenOuter.DeleteObject();
	pDialog->ReleaseDC(pDC);
}

void CShowTrack::DrawPoint(T_POINT_3D tPoint, CDialog *pDialog, COLORREF crColor, int dRatio)
{
	int dDrawX,dDrawY;
	RealCoorToDrawCoor(tPoint.x, tPoint.y, m_dDrawGlobalRatio, m_nDrawGlobalOriginX, m_nDrawGlobalOriginY, dDrawX, dDrawY);
	CDC *pDC = pDialog->GetDC();
	CPen m_cPenOuter;
	m_cPenOuter.CreatePen(PS_SOLID, 1, crColor);
	pDC->SelectObject(m_cPenOuter);
	CBrush pBrush(crColor);
	pDC->SelectObject(pBrush);
	pDC->Ellipse(dDrawX - dRatio, dDrawY - dRatio, dDrawX + dRatio, dDrawY + dRatio);
	m_cPenOuter.DeleteObject();
	pDialog->ReleaseDC(pDC);
}

void CShowTrack::DrawTrack(std::vector <T_POINT_3D> vtTrack, CDialog *pDialog, COLORREF crColor, int nLineType)
{
	CDC *pDC = pDialog->GetDC();
	CPen m_cPenOuter;
	m_cPenOuter.CreatePen(PS_SOLID, 1, crColor);
	pDC->SelectObject(m_cPenOuter);
	int nPartNo;
	int m_nPartNum = vtTrack.size() - 1;
	for (nPartNo = 0; nPartNo < m_nPartNum; nPartNo++)
	{
		double dStartX = vtTrack[nPartNo+1].x;
		double dStartY = vtTrack[nPartNo+1].y;
		double dEndX = vtTrack[nPartNo].x;
		double dEndY = vtTrack[nPartNo].y;
		DrawLine(dStartX, dStartY, dEndX, dEndY, m_dDrawGlobalRatio, m_nDrawGlobalOriginX, m_nDrawGlobalOriginY, pDC, nLineType);
	}	
}

void CShowTrack::DrawLegends(std::vector <T_LEGEND> vtLegends, int xDest, int yDest, CDialog *pDialog)
{
	CDC *pDC = pDialog->GetDC();
	CFont newfont;
	CFont *oldFont;
	newfont.CreateFont(16, 8, 0, 0, FW_NORMAL, FALSE, FALSE, 0, ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, _T("宋体"));
	oldFont = pDC->SelectObject(&newfont);
	for (int i = 0; i < vtLegends.size(); i++)
	{
		int dRatio = 6;
		CPen m_cPenOuter;
		m_cPenOuter.CreatePen(PS_SOLID, 1, vtLegends[i].rgb);
		pDC->SelectObject(m_cPenOuter);
		CBrush pBrush(vtLegends[i].rgb);
		pDC->SelectObject(pBrush);
		pDC->Ellipse(xDest + 8 - dRatio, yDest + 8 + 16 * i - dRatio, xDest + 8 + dRatio, yDest + 8 + 16 * i + dRatio);
		m_cPenOuter.DeleteObject();		
		pDC->SetTextColor(vtLegends[i].rgb);
		pDC->SetBkMode(TRANSPARENT);
		pDC->TextOut(xDest + 16, yDest + 16*i, vtLegends[i].strLegend);
	}	
	pDC->SelectObject(oldFont);
	newfont.DeleteObject();
	pDialog->ReleaseDC(pDC);
}

void CShowTrack::DrawGraph(std::vector <T_DXF_ENTITY_PARA> &vtGraph, CDC *pDC, double dMinRealX, double dMinRealY,
	double dDrawRatio, int nDrawOriginX, int nDrawOriginY, int nLineType)
{
	for (int nEntityNo = 0; nEntityNo < vtGraph.size(); nEntityNo++)//nEntityNo < vtGraph.size()
	{
		if (E_DXF_ENTITY_TYPE_LINE == vtGraph[nEntityNo].eEntityType)
		{
			double dStartX = vtGraph[nEntityNo].dStartX - dMinRealX;
			double dStratY = vtGraph[nEntityNo].dStartY - dMinRealY;
			double dEndX = vtGraph[nEntityNo].dEndX - dMinRealX;
			double dEndY = vtGraph[nEntityNo].dEndY - dMinRealY;
			DrawLine(dStartX, dStratY, dEndX, dEndY, dDrawRatio, nDrawOriginX, nDrawOriginY, pDC, nLineType);
		}
		else
		{
			int nDrawCenterX = vtGraph[nEntityNo].dCenterX - dMinRealX;
			int nDrawCenterY = vtGraph[nEntityNo].dCenterY - dMinRealY;
			int nDrawStartX = vtGraph[nEntityNo].dStartX - dMinRealX;
			int nDrawStartY = vtGraph[nEntityNo].dStartY - dMinRealY;
			int nDrawEndX = vtGraph[nEntityNo].dEndX - dMinRealX;
			int nDrawEndY = vtGraph[nEntityNo].dEndY - dMinRealY;

			double dDis = sqrt(double((nDrawStartX - nDrawCenterX)*(nDrawStartX - nDrawCenterX) + (nDrawStartY - nDrawCenterY)*(nDrawStartY - nDrawCenterY)));
			if (dDis * dDrawRatio > 2.0)
			{
				DrawArc(nDrawCenterX, nDrawCenterY, nDrawStartX, nDrawStartY, nDrawEndX, nDrawEndY,
					vtGraph[nEntityNo].eEntityType, dDrawRatio, nDrawOriginX, nDrawOriginY, pDC, nLineType);
			}
		}
	}
}

void CShowTrack::SetDrawRegionGlobal(CRect rect)
{
	m_rectDrawRegionGlobal = rect;

	m_nDrawGlobalOriginX = m_rectDrawRegionGlobal.right;
	m_nDrawGlobalOriginY = m_rectDrawRegionGlobal.top;

	m_dMinRealX = 0;
	m_dMinRealY = 0;
}

void CShowTrack::GetDrawGlobalRatio(std::vector <T_POINT_3D> vtTrack, double &dRatio)
{
	int nSize = vtTrack.size();
	double dMinX, dMinY, dMaxX, dMaxY;
	dMinX = vtTrack[0].x;
	dMaxX = vtTrack[0].x;
	dMinY = vtTrack[0].y;
	dMaxY = vtTrack[0].y;
	for (int i = 1; i < nSize; i++)
	{
		dMinX = min(dMinX, vtTrack[i].x);
		dMaxX = max(dMaxX, vtTrack[i].x);
		dMinY = min(dMinY, vtTrack[i].y);
		dMaxY = max(dMaxY, vtTrack[i].y);
	}
	m_dMinRealX = dMinX;
	m_dMinRealY = dMinY;
	dRatio = min(m_rectDrawRegionGlobal.Width() / (dMaxX - dMinX),
		m_rectDrawRegionGlobal.Height() / (dMaxY - dMinY));
}

void CShowTrack::SetDrawGlobalRatio(double dDrawGlobalRatio)
{
	m_dDrawGlobalRatio = dDrawGlobalRatio;
}


void CShowTrack::ResizeImage(IplImage* img, IplImage* &dst, double dRatio, T_POINT_2D &tCenter)
{
	if (dRatio > 1.0)
	{
		// 读取图片的宽和高
		int w = img->width;
		int h = img->height;
		// 放大裁剪后图片的宽和高相当于原图片的数值
		double nw = w / dRatio;
		double nh = h / dRatio;
		//限制宽度
		if (tCenter.x - nw / 2.0 < 0)
		{
			tCenter.x = nw / 2.0;
		}
		else if (tCenter.x + nw / 2.0 > w)
		{
			tCenter.x = w - nw / 2.0 - 1;
		}
		//限制高度
		if (tCenter.y - nh / 2.0 < 0)
		{
			tCenter.y = nh / 2.0;
		}
		else if (tCenter.y + nh / 2.0 > h)
		{
			tCenter.y = h - nh / 2.0 - 1;
		}
		// 设置 img 的 ROI 区域
		m_rectROI.left = tCenter.x - nw / 2.0;
		m_rectROI.top = tCenter.y - nh / 2.0;
		m_rectROI.right = m_rectROI.left + nw;
		m_rectROI.bottom = m_rectROI.top + nh;
		cvSetImageROI(img, cvRect(tCenter.x - nw / 2.0, tCenter.y - nh / 2.0, nw, nh));
		// 创建新图片
		dst = cvCreateImage(cvSize(nw, nh), img->depth, img->nChannels);
		// ROI区域内图片复制到新图片
		cvCopy(img, dst, 0);
		// 重置 img 的 ROI
		cvResetImageROI(img);
	}
	else if (dRatio > 0.0)
	{
		m_rectShowWholeImage.right = (m_rectShowWholeImage.right - m_rectShowWholeImage.left) * dRatio + m_rectShowWholeImage.left;
		m_rectShowWholeImage.bottom = (m_rectShowWholeImage.bottom - m_rectShowWholeImage.top) * dRatio + m_rectShowWholeImage.top;
	}
}


void CShowTrack::GetImageROI(CRect &cRect)
{
	cRect = m_rectROI;
}

void CShowTrack::SetImageROI(CvRect &cRect)
{
	m_rectROI.left = cRect.x;
	m_rectROI.top = cRect.y;
	m_rectROI.right = cRect.x + cRect.width;
	m_rectROI.bottom = cRect.y + cRect.height;
}

void CShowTrack::ResizeTrack(double dRatio, T_POINT_2D &tCenter)
{
	//重设原点
	CRect rectDrawRegionGlobal;
	if (dRatio == 1.0 || dRatio <= 0.0)
	{
		return;
	}
	if (dRatio > 1.0)
	{
		rectDrawRegionGlobal.left = m_rectDrawRegionGlobal.left + (dRatio - 1) * m_rectDrawRegionGlobal.Width() / 2.0 + dRatio * (m_rectDrawRegionGlobal.Width() / 2.0 + m_rectShowWholeImage.left - tCenter.x);
		rectDrawRegionGlobal.top = m_rectDrawRegionGlobal.top - (dRatio - 1) * m_rectDrawRegionGlobal.Height() / 2.0 + dRatio * (m_rectDrawRegionGlobal.Height() / 2.0 + m_rectShowWholeImage.top - tCenter.y);
		rectDrawRegionGlobal.right = m_rectDrawRegionGlobal.right + (dRatio - 1) * m_rectDrawRegionGlobal.Width() / 2.0 + dRatio * (m_rectDrawRegionGlobal.Width() / 2.0 + m_rectShowWholeImage.left - tCenter.x);
		rectDrawRegionGlobal.bottom = m_rectDrawRegionGlobal.bottom - (dRatio - 1) * m_rectDrawRegionGlobal.Height() / 2.0 + dRatio * (m_rectDrawRegionGlobal.Height() / 2.0 + m_rectShowWholeImage.top - tCenter.y);
	}
	else if (dRatio > 0.0)
	{
		rectDrawRegionGlobal.left = m_rectDrawRegionGlobal.left - m_rectShowWholeImage.Width() * (1 / dRatio - 1);
		rectDrawRegionGlobal.top = m_rectDrawRegionGlobal.top;
		rectDrawRegionGlobal.right = rectDrawRegionGlobal.left + m_rectDrawRegionGlobal.Width();
		rectDrawRegionGlobal.bottom = rectDrawRegionGlobal.top + m_rectDrawRegionGlobal.Height();
	}
	SetDrawRegionGlobal(rectDrawRegionGlobal);
	//重设比例
	m_dDrawGlobalRatio *= dRatio;
}

