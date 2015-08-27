
// SSDRView.cpp : CSSDRView 类的实现
//

#include "stdafx.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "SSDR.h"
#endif
#include "io.h"
#include "fcntl.h"
#include "SSDRDoc.h"
#include "SSDRView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CSSDRView

IMPLEMENT_DYNCREATE(CSSDRView, CView)

BEGIN_MESSAGE_MAP(CSSDRView, CView)
	// 标准打印命令
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CSSDRView::OnFilePrintPreview)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

// CSSDRView 构造/析构

CSSDRView::CSSDRView()
{
	// TODO: 在此处添加构造代码
	InitConsole();
	up=1;
	angle1=90;
	angle2=0;
	dist=3;
	cX=0;
	cY=0.2;
	cZ=0;
	MouseDown=false;
	drawingIndex=0;
	cIndex=0;
	
	motionCluster.loadObjs(CString("obj/horse/"));
	//motionCluster.Objs.at(0).writeToFile("horse-ref.obj");
	//motionCluster.loadObjs(CString("obj/cat/"));
	motionCluster.generateClusters();
	motionCluster.SSDRstep();
	motionCluster.TopologyReconstr();
	motionCluster.IterativeRigging();
	motionCluster.writeFile(CString("output.txt"));
	motionCluster.writeTestFile(CString("outputTest.txt"));

}

CSSDRView::~CSSDRView()
{
}

BOOL CSSDRView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CView::PreCreateWindow(cs);
}

void CSSDRView::InitConsole()
{
	int nRet= 0;
	FILE* fp;
	AllocConsole();
	nRet= _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	fp = _fdopen(nRet, "w");
	*stdout = *fp;
	setvbuf(stdout, NULL, _IONBF, 0);
}
// CSSDRView 绘制

void CSSDRView::OnDraw(CDC* /*pDC*/)
{
	CSSDRDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: 在此处为本机数据添加绘制代码
	// 清除颜色
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// 绘制场景
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	{
		// 绘制坐标系
		//glDepthMask(false);
		DrawScenes();
		//glDepthMask(true);
	}
	glPopMatrix();
	// 交换缓冲区
	SwapBuffers(wglGetCurrentDC());
}

void CSSDRView::DrawScenes()
{
	GLfloat mat_diffuse[] = {1.0,0,0,1};
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
	for (int i=0;i<motionCluster.MSTedgeSet.size();i++)
	{
		EdgeNode& edge=motionCluster.MSTedgeSet.at(i);
		Vector3d center=(motionCluster.joint[edge.v1][edge.v2]).center;
		glPushMatrix();
			glTranslatef(center(0),center(1),center(2));
			glutSolidSphere(0.003,20,20);
		glPopMatrix();
	}

	//motionCluster.Objs.at(drawingIndex).glDraw(motionCluster.Vtest);
	motionCluster.Objs.at(drawingIndex).glDraw();
}

// CSSDRView 打印

void CSSDRView::OnFilePrintPreview()
{
#ifndef SHARED_HANDLERS
	AFXPrintPreview(this);
#endif
}

BOOL CSSDRView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 默认准备
	return DoPreparePrinting(pInfo);
}

void CSSDRView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加额外的打印前进行的初始化过程
}

void CSSDRView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加打印后进行的清理过程
}

void CSSDRView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
	OnContextMenu(this, point);
}

void CSSDRView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CSSDRView 诊断

#ifdef _DEBUG
void CSSDRView::AssertValid() const
{
	CView::AssertValid();
}

void CSSDRView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CSSDRDoc* CSSDRView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CSSDRDoc)));
	return (CSSDRDoc*)m_pDocument;
}
#endif //_DEBUG


// CSSDRView 消息处理程序


int CSSDRView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	m_pDC = new CClientDC(this);
	ASSERT(m_pDC != NULL);
	// 选择像素格式
	if(!bSetDCPixelFormat()) return -1;
	// 创建渲染环境, 并使它成为当前渲染环境
	m_hRC = wglCreateContext(m_pDC->GetSafeHdc());
	wglMakeCurrent(m_pDC->GetSafeHdc(), m_hRC);



	
	return 0;
}

BOOL CSSDRView::bSetDCPixelFormat()
{
	// 设置像素格式
	static PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR), // 结构的大小
		1, // 结构的版本
		PFD_DRAW_TO_WINDOW | // 在窗口(而不是位图)中绘图
		PFD_SUPPORT_OPENGL | // 支持在窗口中进行OpenGL调用
		PFD_DOUBLEBUFFER, // 双缓冲模式
		PFD_TYPE_RGBA, // RGBA颜色模式
		32, // 需要32位颜色
		0, 0, 0, 0, 0, 0, // 不用于选择模式
		0, 0, // 不用于选择模式
		0, 0, 0, 0, 0, // 不用于选择模式
		16, // 深度缓冲区的大小
		0, // 在此不使用
		0, // 在此不使用
		0, // 在此不使用
		0, // 在此不使用
		0, 0, 0 // 在此不使用
	};
	// 选择一种与pfd所描述的最匹配的像素格式
	int nPixelFormat = ChoosePixelFormat(m_pDC->GetSafeHdc(), &pfd);
	if(0 == nPixelFormat) return false;
	// 为设备环境设置像素格式
	return SetPixelFormat(m_pDC->GetSafeHdc(), nPixelFormat, &pfd);
}

void CSSDRView::OnDestroy()
{
	CView::OnDestroy();

	// TODO: 在此处添加消息处理程序代码
	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(m_hRC);
	delete m_pDC;
}


void CSSDRView::OnInitialUpdate()
{
	CView::OnInitialUpdate();

	// TODO: 在此添加专用代码和/或调用基类
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_LIGHTING);
	GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_pos[] = {0,0,5,1};

	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, white);
	glEnable(GL_LIGHT0);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //指定混合函数  
	glEnable(GL_BLEND);     //启用混合状态  
	glShadeModel(GL_SMOOTH);
}


void CSSDRView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	// TODO: 在此处添加消息处理程序代码
	// 设置视口
	resize(cx,cy);
	updateCam();
}

void CSSDRView::resize(int x,int y)
{
	glViewport(0, 0, x, y);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)x/(GLfloat)y, 0.001, 5000.0);
}

void CSSDRView::updateCam()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(dist*cos(angle2/180*Pi)*cos(angle1/180*Pi)+cX, dist*sin(angle2/180*Pi)+cY,
		dist*cos(angle2/180*Pi)*sin(angle1/180*Pi)+cZ, cX, cY, cZ, 0.0, up, 0.0);
}

void CSSDRView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	MouseDown=true;
	CView::OnLButtonDown(nFlags, point);
}


void CSSDRView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	MouseDown=false;
	lastPoint=CPoint(0,0);
	CView::OnLButtonUp(nFlags, point);
}


void CSSDRView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if(MouseDown){
		if (lastPoint!=CPoint(0,0))
		{
			angle1+=(point.x-lastPoint.x)*up;
			angle2+=(point.y-lastPoint.y)*up;
			if (angle2>90)
			{
				angle1+=180;
				angle2=180-angle2;
				up=-up;
			}
			if(angle2<-90)
			{
				angle1+=180;
				angle2=-180-angle2;
				up=-up;
			}
			updateCam();
			CPaintDC dc(this);
			OnDraw(&dc);
		} 
		if(lastPoint!=point)
			lastPoint=point;		
	}

	CView::OnMouseMove(nFlags, point);
}


BOOL CSSDRView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	dist-=zDelta/1000.0;
	if (dist<0.1)
		dist=0.1;

	updateCam();
	CPaintDC dc(this);
	OnDraw(&dc);
	return CView::OnMouseWheel(nFlags, zDelta, pt);
}


BOOL CSSDRView::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 在此添加专用代码和/或调用基类
	if (WM_KEYFIRST <= pMsg->message && pMsg->message <= WM_KEYLAST)
	{
		//判断是否按下键盘Enter键
		if(pMsg->wParam=='1')
		{
			//Do anything what you want to
			drawingIndex=1;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='2')
		{
			//Do anything what you want to
			drawingIndex=2;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='3')
		{
			//Do anything what you want to
			drawingIndex=3;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='4')
		{
			//Do anything what you want to
			drawingIndex=4;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='5')
		{
			//Do anything what you want to
			drawingIndex=5;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='6')
		{
			//Do anything what you want to
			drawingIndex=6;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='7')
		{
			//Do anything what you want to
			drawingIndex=7;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='8')
		{
			//Do anything what you want to
			drawingIndex=8;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='a')
		{
			//Do anything what you want to
			drawingIndex=0;
			CPaintDC dc(this);
			OnDraw(&dc);
		}
		if(pMsg->wParam=='q')
		{
			//Do anything what you want to
			cIndex++;
			if (cIndex>=motionCluster.boneNum)
			{
				cIndex=0;
			}
			CPaintDC dc(this);
			OnDraw(&dc);
		}
	}
	return CView::PreTranslateMessage(pMsg);
}
