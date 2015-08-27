#pragma once
#include "MyVertex.h"
struct Face
{
	int first;
	int second;
	int third;
};
class ObjModel
{
public:
	MyVertex* vertex;
	Face* face;
	Vector3d* normal;
	int vertexNum;
	int faceNum;
	GLfloat color[64][4];
public:	
	ObjModel(void);
	ObjModel(CString fileName);
	ObjModel(ObjModel& obj);
	~ObjModel(void);
	ObjModel& operator=(ObjModel& obj);
	void loadObjFromFile(CString fileName);
	void loadObjFromFileEx(CString fileName);
	void glDraw();
	void glDraw(int clndex);
	void glDraw(double* V);
	double calDist(ObjModel& refObj);
	void writeToFile(CString fileName);
};

