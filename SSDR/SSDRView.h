
// SSDRView.h : CSSDRView 类的接口
//

#pragma once
#include "ObjModel.h"
#include "MotionCluster.h"
using namespace std;

class CSSDRView : public CView
{
private:
	HGLRC m_hRC;
	CClientDC* m_pDC;
	double angle1;
	double angle2;
	double dist;
	double cX;
	double cY;
	double cZ;
	int up;
	bool MouseDown;
	CPoint lastPoint;
	vector<ObjModel> Objs;
	int drawingIndex;
	MotionCluster motionCluster;
	int cIndex;

protected: // 仅从序列化创建
	CSSDRView();
	DECLARE_DYNCREATE(CSSDRView)

// 特性
public:
	CSSDRDoc* GetDocument() const;

// 操作
public:

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);
	BOOL bSetDCPixelFormat();
// 实现
public:
	virtual ~CSSDRView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	virtual void OnInitialUpdate();
	afx_msg void OnSize(UINT nType, int cx, int cy);

public:
	void DrawScenes();
	void resize(int x,int y);
	void updateCam();
	void InitConsole();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	virtual BOOL PreTranslateMessage(MSG* pMsg);
};

#ifndef _DEBUG  // SSDRView.cpp 中的调试版本
inline CSSDRDoc* CSSDRView::GetDocument() const
   { return reinterpret_cast<CSSDRDoc*>(m_pDocument); }
#endif

