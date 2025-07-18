// ChangeScanLineParam.cpp: 实现文件
//

#include "StdAfx.h"
#include "res\resource.h"
#include "ChangeScanLineParam.h"
#include "XiAlgorithm.h"
using namespace std;

IMPLEMENT_DYNAMIC(ChangeScanLineParam, CDialogEx)

ChangeScanLineParam::ChangeScanLineParam(CUnit *pUnit, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_CHANGE_SCANPARAM, pParent)
	, m_pUnit(pUnit)
	, m_bRButtonIsDown(false)
	, m_bIsRunning(false)
{
	
}

ChangeScanLineParam::~ChangeScanLineParam()
{
	m_pUnit = NULL;
}

void ChangeScanLineParam::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(ChangeScanLineParam, CDialogEx)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_PAINT()
	ON_WM_MOUSEMOVE()
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN2, &ChangeScanLineParam::OnEnChangeEdit2)
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN3, &ChangeScanLineParam::OnEnChangeEdit3)
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN4, &ChangeScanLineParam::OnEnChangeEdit4)
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN5, &ChangeScanLineParam::OnEnChangeEdit5)
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN6, &ChangeScanLineParam::OnEnChangeEdit6)
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN7, &ChangeScanLineParam::OnEnChangeEdit7)
	ON_EN_CHANGE(IDC_EDIT_LINE_SCAN8, &ChangeScanLineParam::OnEnChangeEdit8)
	ON_BN_CLICKED(IDC_Btn_LINE_SCANOK, &ChangeScanLineParam::OnBnClickedBtnok)
	ON_BN_CLICKED(IDC_Btn_LINE_SCANOK3, &ChangeScanLineParam::OnBnClickedBtnok3)
	ON_BN_CLICKED(IDC_Btn_LINE_SCANOK4, &ChangeScanLineParam::OnBnClickedBtnok4)
	ON_BN_CLICKED(IDC_Btn_LINE_SCANOK5, &ChangeScanLineParam::OnBnClickedBtnok5)
END_MESSAGE_MAP()

// ChangeScanLineParam 消息处理程序
void ChangeScanLineParam::DrawTable()
{
	// 获取该控件画布
	COLORREF colorRef = RGB(192, 250, 233);
	m_ptableDC->FillSolidRect(m_tableRect, colorRef);
	return;
}

void ChangeScanLineParam::DrawScale()
{
	// 设置控件透明
	m_ptableDC->SetBkMode(TRANSPARENT);

	int nScaleDisX = 500; // mm
	int nScaleDisY = 1000; // mm

	int yNum = m_rTableLength / nScaleDisY;
	int xNum = m_rTableWidth / nScaleDisX;

	double yScale = (double)m_tableWidth / (double)m_rTableLength * (double)nScaleDisY;  // 每个像素代表距离 * nScaleDis
	double xScale = (double)m_tableHeight / (double)m_rTableWidth * (double)nScaleDisX;

	//画X方向
	m_ptableDC->MoveTo(m_tableSx, m_tableEy);
	m_ptableDC->LineTo(m_tableSx, m_tableSy);
	//画Y方向
	m_ptableDC->MoveTo(m_tableSx, m_tableEy);
	m_ptableDC->LineTo(m_tableEx, m_tableEy);

	//画X正刻度
	m_ptableDC->MoveTo(m_tableSx, m_tableHeight / 2);
	m_ptableDC->LineTo(m_tableSx + m_tableWidth, m_tableHeight / 2);
	m_ptableDC->TextOutA(m_tableSx + 10, m_tableHeight / 2, "0.0");

	//画y正刻度
	int n = 0;
	for (int j = 0; j <= m_tableWidth; j += yScale)
	{
		CString cStr;
		cStr.Format("%.1lfm", (double)n * (double)nScaleDisY / 1000.0 + dMinScanPos / 1000.0);
		m_ptableDC->MoveTo(j, m_tableEy);
		m_ptableDC->LineTo(j, m_tableEy - 10);
		//绘出文字
		int nOffsetPix = 0 == j ? 0 : -20;
		m_ptableDC->TextOutA(j + nOffsetPix, m_tableHeight - 25, cStr);
		n++;
	}
}

void ChangeScanLineParam::DrawArrow()
{
	CPoint tPtnS((m_ScanStartLocation - dMinScanPos) / rStoW, m_tableHeight / 2 - m_ScanOffsetX / (m_rTableWidth / m_tableHeight));
	CPoint tPtnE((m_ScanEndLocation - dMinScanPos) / rStoW, m_tableHeight / 2 - m_ScanOffsetX / (m_rTableWidth / m_tableHeight));
	// 画箭头
	XiAlgorithm alg;
	int nRadius = 5;
	double dAngle1 = 30.0;
	double dAngle2 = 20.0;
	double dDis1 = 20.0;
	double dDis2 = 15.0;

	CPen cNewPen, * poldPen;
	CBrush cNewBrush, * pOldBrush;
	cNewPen.CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
	poldPen = m_ptableDC->SelectObject(&cNewPen);
	CRect tRectS(tPtnS.x - nRadius, tPtnS.y - nRadius, tPtnS.x + nRadius, tPtnS.y + nRadius);
	CRect tRectE(tPtnE.x - nRadius, tPtnE.y - nRadius, tPtnE.x + nRadius, tPtnE.y + nRadius);
	m_ptableDC->Ellipse(tRectS);
	m_ptableDC->Ellipse(tRectE);

	cNewPen.DeleteObject();
	cNewPen.CreatePen(PS_SOLID, 1, RGB(255, 0, 0));
	m_ptableDC->SelectObject(&cNewPen);
	cNewBrush.CreateSolidBrush(RGB(255, 0, 0));
	pOldBrush = m_ptableDC->SelectObject(&cNewBrush);

	double dDirAngle = alg.CalcArcAngle(tPtnS.x - tPtnE.x, tPtnS.y - tPtnE.y);
	CPoint aPtns[6] = { 0 };
	aPtns[0] = tPtnS;
	aPtns[1] = tPtnE;
	aPtns[1].x += (int)(dDis2 * CosD(dDirAngle + dAngle2));
	aPtns[1].y += (int)(dDis2 * SinD(dDirAngle + dAngle2));
	aPtns[2] = tPtnE;
	aPtns[2].x += (int)(dDis1 * CosD(dDirAngle + dAngle1));
	aPtns[2].y += (int)(dDis1 * SinD(dDirAngle + dAngle1));
	aPtns[3] = tPtnE;
	aPtns[4] = tPtnE;
	aPtns[4].x += (int)(dDis1 * CosD(dDirAngle - dAngle1));
	aPtns[4].y += (int)(dDis1 * SinD(dDirAngle - dAngle1));
	aPtns[5] = tPtnE;
	aPtns[5].x += (int)(dDis2 * CosD(dDirAngle - dAngle2));
	aPtns[5].y += (int)(dDis2 * SinD(dDirAngle - dAngle2));
	m_ptableDC->Polygon(aPtns, 6);
	m_ptableDC->SelectObject(poldPen);
	m_ptableDC->SelectObject(pOldBrush);
}

void ChangeScanLineParam::LoadScanParam()
{
	double dScanLength = 0;
	int nCurUseTable = 0;
	COPini opini2;
	opini2.SetFileName(DATA_PATH + m_pUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini2.SetSectionName("CurUseTableNo");
	opini2.ReadString("CurUseTableNo", &nCurUseTable);

	CString table;
	table.Format("Table%d", nCurUseTable);

	COPini opini;
	opini.SetFileName(DATA_PATH + m_pUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini.SetSectionName(table);
	opini.ReadString("YMaxCar", &dMaxScanPos);
	opini.ReadString("YMinCar", &dMinScanPos);
	m_rTableLength = (int)fabs(dMaxScanPos - dMinScanPos);
	m_rTableWidth = 1000.0; // X可调整范围正负500.0mm

	opini.ReadString("startpulse.nS", &m_tLineScanPulse.nSPulse);
	opini.ReadString("startpulse.nL", &m_tLineScanPulse.nLPulse);
	opini.ReadString("startpulse.nU", &m_tLineScanPulse.nUPulse);
	opini.ReadString("startpulse.nR", &m_tLineScanPulse.nRPulse);
	opini.ReadString("startpulse.nB", &m_tLineScanPulse.nBPulse);
	opini.ReadString("startpulse.nT", &m_tLineScanPulse.nTPulse);

	opini.ReadString("ScanStartCarLoction", &m_ScanStartLocation);
	opini.ReadString("Scanlength", &dScanLength);
	m_ScanEndLocation = m_ScanStartLocation + dScanLength;

	opini.ReadString("RXScanSize", &m_ScanOffsetRX);
	opini.ReadString("RYScanSize", &m_ScanOffsetRY);
	opini.ReadString("RZScanSize", &m_ScanOffsetRZ);
	opini.ReadString("XScanSize", &m_ScanOffsetX);
	opini.ReadString("ZScanSize", &m_ScanOffsetZ);

	SetDlgItemData(IDC_EDIT_LINE_SCAN2, m_ScanStartLocation, this);
	SetDlgItemData(IDC_EDIT_LINE_SCAN3, m_ScanEndLocation, this);
	SetDlgItemData(IDC_EDIT_LINE_SCAN4, m_ScanOffsetRX, this);
	SetDlgItemData(IDC_EDIT_LINE_SCAN5, m_ScanOffsetZ, this);
	SetDlgItemData(IDC_EDIT_LINE_SCAN6, m_ScanOffsetRY, this);
	SetDlgItemData(IDC_EDIT_LINE_SCAN7, m_ScanOffsetRZ, this);
	SetDlgItemData(IDC_EDIT_LINE_SCAN8, m_ScanOffsetX, this);
}

bool ChangeScanLineParam::CheckRobotOffsetValid(T_ANGLE_PULSE& tScanPulse)
{
	// 检查是否限位 是否可达
	T_ROBOT_COORS tLineScanCoord;
	LoadScanParam();
	m_pUnit->GetRobotCtrl()->RobotKinematics(m_tLineScanPulse, m_pUnit->GetRobotCtrl()->m_tTools.tGunTool, tLineScanCoord);
#ifdef SINGLE_ROBOT
	tLineScanCoord.dX += m_ScanOffsetX;
	tLineScanCoord.dZ += m_ScanOffsetZ * (double)m_pUnit->GetRobotCtrl()->m_nRobotInstallDir;
	tLineScanCoord.dRX += m_ScanOffsetRX;
	tLineScanCoord.dRY += m_ScanOffsetRY;
	tLineScanCoord.dRZ += m_ScanOffsetRZ;
#else
	tLineScanCoord.dY += m_ScanOffsetX; // 临时
	tLineScanCoord.dZ += m_ScanOffsetZ * (double)m_pUnit->GetRobotCtrl()->m_nRobotInstallDir;
	tLineScanCoord.dRX += m_ScanOffsetRX;
	tLineScanCoord.dRY += m_ScanOffsetRY;
	tLineScanCoord.dRZ += m_ScanOffsetRZ;
#endif // SINGLE_ROBOT

	return m_pUnit->GetRobotCtrl()->RobotInverseKinematics(tLineScanCoord, m_tLineScanPulse, m_pUnit->GetRobotCtrl()->m_tTools.tGunTool, tScanPulse);
}

BOOL ChangeScanLineParam::OnInitDialog()
{

	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	// 获取该控件指针
	m_ptablepWnd = GetDlgItem(IDC_STATIC_PICTURE);
	// 把控件的长宽、坐标等信息保存在rect里
	m_ptablepWnd->GetWindowRect(&m_tableRect);
	m_ptablepWnd->ScreenToClient(&m_tableRect);
	// 获取控件信息
	m_tableWidth = m_tableRect.Width();
	m_tableHeight = m_tableRect.Height();
	m_tableSx = m_tableRect.left;
	m_tableSy = m_tableRect.top;
	m_tableEx = m_tableRect.right;
	m_tableEy = m_tableRect.bottom;
	// 获取料台对应控件画布
	m_ptableDC = m_ptablepWnd->GetDC();
	// 读取配置文件
	LoadScanParam();

	// 料台实际长度与控件长度转换关系
	rStoW = (double)m_rTableLength / (double)m_tableWidth;

	m_IsInital = true;
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}

void ChangeScanLineParam::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	CDialogEx::OnLButtonDown(nFlags, point);
}

void ChangeScanLineParam::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和或调用默认值

	CDialogEx::OnLButtonUp(nFlags, point);
}

void ChangeScanLineParam::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和或调用默认值
	m_bRButtonIsDown = true;
	m_ptStart = 0;
	// 光标所在点屏幕坐标转控件坐标
	ClientToScreen(&point);
	m_ptablepWnd->ScreenToClient(&point);

	if (m_tableRect.PtInRect(point))
	{
		m_ptStart = point;
	}

	CDialogEx::OnRButtonDown(nFlags, point);
}

void ChangeScanLineParam::OnRButtonUp(UINT nFlags, CPoint point)
{
	m_bRButtonIsDown = false;
	CDialogEx::OnRButtonUp(nFlags, point);
}

void ChangeScanLineParam::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	ClientToScreen(&point);
	m_ptablepWnd->ScreenToClient(&point);
	if (m_bRButtonIsDown && m_tableRect.PtInRect(point))
	{
		// 保存线扫终点
		m_ptEnd = point;
		m_ScanOffsetX = -1 * (m_ptEnd.y - m_tableHeight / 2) * (m_rTableWidth / m_tableHeight);
		m_ScanStartLocation = m_ptStart.x * rStoW + dMinScanPos;
		m_ScanEndLocation = m_ptEnd.x * rStoW + dMinScanPos;
		// 清除画布
		RedrawWindow(CRect(0, 0, m_tableWidth, m_tableHeight));
		// 重绘料台
		DrawTable();
		DrawScale();
		DrawArrow();

		SetDlgItemData(IDC_EDIT_LINE_SCAN2, m_ScanStartLocation, this);
		SetDlgItemData(IDC_EDIT_LINE_SCAN3, m_ScanEndLocation, this);
		SetDlgItemData(IDC_EDIT_LINE_SCAN8, m_ScanOffsetX, this);
	}
	CDialogEx::OnMouseMove(nFlags, point);
}

void ChangeScanLineParam::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: 在此处添加消息处理程序代码
	if (true == m_IsInital)
	{
		DrawTable();
		DrawScale();
		DrawArrow();
		m_IsInital = false;
	}
	CDialogEx::OnPaint();
}

void ChangeScanLineParam::OnEnChangeEdit8()
{
	if (false == m_bRButtonIsDown)
	{
		double dOldScanOffsetX = m_ScanOffsetX;
		GetDlgItemData(IDC_EDIT_LINE_SCAN8, m_ScanOffsetX, this);
		if (fabs(m_ScanOffsetX) > fabs(m_rTableWidth / 2.0))
		{
			CString sData;
			sData.Format("%.1lf", dOldScanOffsetX);
			SetDlgItemData(IDC_EDIT_LINE_SCAN8, sData, this);
			sData.Format("线扫X方向调整有效输入:%.1lf - %.1lf", -fabs(m_rTableWidth / 2.0), fabs(m_rTableWidth / 2.0));
			AfxMessageBox(sData);
			return;
		}
		DrawTable();
		DrawScale();
		DrawArrow();
	}
}

void ChangeScanLineParam::OnEnChangeEdit2()
{
	if (false == m_bRButtonIsDown)
	{
		double dOldScanStartLocation = m_ScanStartLocation;
		GetDlgItemData(IDC_EDIT_LINE_SCAN2, m_ScanStartLocation, this);
		if ((m_ScanStartLocation > dMaxScanPos) || (m_ScanStartLocation < dMinScanPos))
		{
			CString sData;
			sData.Format("%.1lf", dOldScanStartLocation);
			SetDlgItemData(IDC_EDIT_LINE_SCAN2, sData, this);
			sData.Format("线扫起点位置有效输入:%.1lf - %.1lf", dMinScanPos, dMaxScanPos);
			//已修改
			XUI::MesBox::PopInfo("线扫起点位置有效输入:{0}-{1}", dMinScanPos, dMaxScanPos);
			//AfxMessageBox(sData);
			return;
		}
		DrawTable();
		DrawScale();
		DrawArrow();
	}
}

void ChangeScanLineParam::OnEnChangeEdit3()
{
	if (false == m_bRButtonIsDown)
	{
		double dOldScanEndLocation = m_ScanEndLocation;
		GetDlgItemData(IDC_EDIT_LINE_SCAN3, m_ScanEndLocation, this);
		if ((m_ScanEndLocation > dMaxScanPos) || (m_ScanEndLocation < dMinScanPos))
		{
			CString sData;
			sData.Format("%.1lf", dOldScanEndLocation);
			SetDlgItemData(IDC_EDIT_LINE_SCAN3, sData, this);
			sData.Format("线扫终点位置有效输入:%.1lf - %.1lf", dMinScanPos, dMaxScanPos);
			//已修改
			XUI::MesBox::PopInfo("线扫终点位置有效输入:{0}-{1}", dMinScanPos, dMaxScanPos);
			//AfxMessageBox(sData);
			return;
		}
		DrawTable();
		DrawScale();
		DrawArrow();
	}
}

void ChangeScanLineParam::OnEnChangeEdit5()
{
	double dThresholdScanOffsetZ = 500.0;
	double dOldScanOffsetZ = m_ScanOffsetZ;
	GetDlgItemData(IDC_EDIT_LINE_SCAN5, m_ScanOffsetZ, this);
	if (fabs(m_ScanOffsetZ) > fabs(dThresholdScanOffsetZ))
	{
		CString sData;
		sData.Format("%.1lf", dOldScanOffsetZ);
		SetDlgItemData(IDC_EDIT_LINE_SCAN5, sData, this);
		sData.Format("线扫高度调整有效输入:%.1lf - %.1lf", -fabs(dThresholdScanOffsetZ), fabs(dThresholdScanOffsetZ));
		AfxMessageBox(sData);
		return;
	}
}

void ChangeScanLineParam::OnEnChangeEdit4()
{
	double dThreshold = 30.0;
	double dOldScanOffsetRx = m_ScanOffsetRX;
	GetDlgItemData(IDC_EDIT_LINE_SCAN4, m_ScanOffsetRX, this);
	if (fabs(m_ScanOffsetRX) > fabs(dThreshold))
	{
		CString sData;
		sData.Format("%.1lf", dOldScanOffsetRx);
		SetDlgItemData(IDC_EDIT_LINE_SCAN4, sData, this);
		sData.Format("线扫Rx调整有效输入:%.1lf - %.1lf", -fabs(dThreshold), fabs(dThreshold));
		AfxMessageBox(sData);
		return;
	}
}

void ChangeScanLineParam::OnEnChangeEdit6()
{
	double dThreshold = 30.0;
	double dOldScanOffsetRy = m_ScanOffsetRY;
	GetDlgItemData(IDC_EDIT_LINE_SCAN6, m_ScanOffsetRY, this);
	if (fabs(m_ScanOffsetRY) > fabs(dThreshold))
	{
		CString sData;
		sData.Format("%.1lf", dOldScanOffsetRy);
		SetDlgItemData(IDC_EDIT_LINE_SCAN6, sData, this);
		sData.Format("线扫Ry调整有效输入:%.1lf - %.1lf", -fabs(dThreshold), fabs(dThreshold));
		AfxMessageBox(sData);
		return;
	}
}

void ChangeScanLineParam::OnEnChangeEdit7()
{
	double dThreshold = 30.0;
	double dOldScanOffsetRz = m_ScanOffsetRZ;
	GetDlgItemData(IDC_EDIT_LINE_SCAN7, m_ScanOffsetRZ, this);
	if (fabs(m_ScanOffsetRZ) > fabs(dThreshold))
	{
		CString sData;
		sData.Format("%.1lf", dOldScanOffsetRz);
		SetDlgItemData(IDC_EDIT_LINE_SCAN7, sData, this);
		sData.Format("线扫Rz调整有效输入:%.1lf - %.1lf", -fabs(dThreshold), fabs(dThreshold));
		AfxMessageBox(sData);
		return;
	}
}

void ChangeScanLineParam::OnBnClickedBtnok()
{
	// TODO: 在此添加控件通知处理程序代码
	GetDlgItemData(IDC_EDIT_LINE_SCAN2, m_ScanStartLocation, this);
	GetDlgItemData(IDC_EDIT_LINE_SCAN3, m_ScanEndLocation, this);
	GetDlgItemData(IDC_EDIT_LINE_SCAN8, m_ScanOffsetX, this);
	GetDlgItemData(IDC_EDIT_LINE_SCAN5, m_ScanOffsetZ, this);
	GetDlgItemData(IDC_EDIT_LINE_SCAN4, m_ScanOffsetRX, this);
	GetDlgItemData(IDC_EDIT_LINE_SCAN6, m_ScanOffsetRY, this);
	GetDlgItemData(IDC_EDIT_LINE_SCAN7, m_ScanOffsetRZ, this);

	T_ANGLE_PULSE tRealScanPulse;
	//if (!CheckRobotOffsetValid(tRealScanPulse))
	//{
	//	XiMessageBox("调整后的扫描坐标不可达，请重新修改后保存！");
	//	return;
	//}
	if (labs(m_ScanEndLocation - m_ScanStartLocation) < 200)
	{
		XUI::MesBox::PopOkCancel("线扫扫描距离{0}mm过短 请保证扫描距离大于200mm", m_ScanEndLocation - m_ScanStartLocation);
		return;
	}
	int nCurUseTable = 0;
	COPini opini2;
	opini2.SetFileName(DATA_PATH + m_pUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini2.SetSectionName("CurUseTableNo");
	opini2.ReadString("CurUseTableNo", &nCurUseTable);

	CString table;
	table.Format("Table%d", nCurUseTable);

	COPini opini;
	opini.SetFileName(DATA_PATH + m_pUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini.SetSectionName(table);
	opini.WriteString("TableLength", m_rTableLength);
	opini.WriteString("TableWidth", m_rTableWidth);
	opini.WriteString("ScanStartCarLoction", m_ScanStartLocation);
	opini.WriteString("Scanlength", m_ScanEndLocation - m_ScanStartLocation);
	opini.WriteString("RXScanSize", m_ScanOffsetRX);
	opini.WriteString("RYScanSize", m_ScanOffsetRY);
	opini.WriteString("RZScanSize", m_ScanOffsetRZ);	
	opini.WriteString("XScanSize", m_ScanOffsetX);
	opini.WriteString("ZScanSize", m_ScanOffsetZ);

	XiMessageBox("参数修改完成");
	m_IsInital = true;

	//添加给新线扫
	//opini.SetFileName(".\\LineScan\\Gantry\\Table.ini");
	if (nCurUseTable == 0)
		opini.SetFileName(".\\LineScan\\GantryLeft\\Table.ini");
	else
		opini.SetFileName(".\\LineScan\\GantryRight\\Table.ini");
	opini.SetSectionName("Base");
	opini.WriteString("MoveVector_BX", m_ScanEndLocation > m_ScanStartLocation ? 1 : -1);
	opini.SetSectionName("Section0");
	opini.WriteString("LineScanStartPos_BX", m_ScanStartLocation);
	opini.WriteString("ScanDis", fabs(m_ScanEndLocation - m_ScanStartLocation));
	return;
}

void ChangeScanLineParam::OnBnClickedBtnok3()
{
	if (false == m_bIsRunning)
	{
		CHECK_BOOL(m_pUnit->CheckIsReadyRun());
		m_bIsRunning = true;
		T_ANGLE_PULSE tRealScanPulse;
		if (!CheckRobotOffsetValid(tRealScanPulse))
		{
			XiMessageBox("当前偏移位置无法到位！请重新设置！");
			m_bIsRunning = false;
			return;
		}
		tRealScanPulse.lBXPulse = m_ScanStartLocation / m_pUnit->GetRobotCtrl()->m_tAxisUnit.dTPulse;
		T_ROBOT_MOVE_INFO tRobotMoveInfo;
		vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;
		vtRobotMoveInfo.clear();
		tRobotMoveInfo = m_pUnit->GetRobotCtrl()->PVarToRobotMoveInfo(
			1, tRealScanPulse, m_pUnit->GetRobotCtrl()->m_tPulseHighSpeed, MOVJ);
		vtRobotMoveInfo.push_back(tRobotMoveInfo);
		m_pUnit->GetRobotCtrl()->SetMoveValue(vtRobotMoveInfo);
		m_pUnit->GetRobotCtrl()->CallJob("CONTIMOVANY");
		//m_pUnit->GetRobotCtrl()->MoveByJob(tRealScanPulse, m_pUnit->GetRobotCtrl()->m_tPulseHighSpeed, m_pUnit->GetRobotCtrl()->m_nExternalAxleType, "MOVJ");
		m_pUnit->RobotCheckDone();
		m_bIsRunning = false;
	}
}

void ChangeScanLineParam::OnBnClickedBtnok4()
{
	if (false == m_bIsRunning)
	{
		m_bIsRunning = true;
		m_ScanStartLocation;
		int nAxisNo = m_pUnit->m_nLinedScanAxisNo;
		double dMaxExAxisSpeed = m_pUnit->GetMotorCtrl()->GetMaxSpeed(m_pUnit->XYZAxisNoToSoftAxisNo(nAxisNo));
		if (0 != m_pUnit->MoveExAxisFun(m_ScanStartLocation, m_pUnit->GetRobotCtrl()->m_tPulseHighSpeed.dSpeed*3, nAxisNo, dMaxExAxisSpeed))
		{
			m_bIsRunning = false;
			return;
		}
		m_pUnit->WorldCheckRobotDone();
		m_bIsRunning = false;
	}
}

void ChangeScanLineParam::OnBnClickedBtnok5()
{
	if (false == m_bIsRunning)
	{
		m_bIsRunning = true; 
		m_ScanEndLocation;
		int nAxisNo = m_pUnit->m_nLinedScanAxisNo;
		double dMaxExAxisSpeed = m_pUnit->GetMotorCtrl()->GetMaxSpeed(m_pUnit->XYZAxisNoToSoftAxisNo(nAxisNo));
		if (0 != m_pUnit->MoveExAxisFun(m_ScanEndLocation, m_pUnit->GetRobotCtrl()->m_tPulseHighSpeed.dSpeed*3, nAxisNo, dMaxExAxisSpeed))
		{
			m_bIsRunning = false;
			return;
		}
		m_pUnit->WorldCheckRobotDone();
		m_bIsRunning = false;
	}
}
