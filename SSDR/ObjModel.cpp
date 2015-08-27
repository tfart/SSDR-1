#include "stdafx.h"
#include "ObjModel.h"


ObjModel::ObjModel(void)
{
	vertex=NULL;
	normal=NULL;
	face=NULL;
}


ObjModel::~ObjModel(void)
{
	if (vertex!=NULL)
	{
		delete[] vertex;
		delete[] normal;
		delete[] face;
		vertex=NULL;
		normal=NULL;
		face=NULL;
	}
}

ObjModel::ObjModel(CString fileName)
{
	vertex=NULL;
	normal=NULL;
	face=NULL;
	loadObjFromFile(fileName);
}

ObjModel::ObjModel(ObjModel& obj)
{
	vertexNum=obj.vertexNum;
	faceNum=obj.faceNum;
	vertex=obj.vertex;
	face=obj.face;
	normal=obj.normal;
	obj.vertex=NULL;
	obj.face=NULL;
	obj.normal=NULL;
}

ObjModel& ObjModel::operator=(ObjModel& obj)
{
	vertexNum=obj.vertexNum;
	faceNum=obj.faceNum;
	vertex=obj.vertex;
	face=obj.face;
	normal=obj.normal;
	obj.vertex=NULL;
	obj.face=NULL;
	obj.normal=NULL;

	return *this;
}

void ObjModel::loadObjFromFile(CString fileName)
{
	char c;
	double x;
	double y;
	double z;
	int tmp;
	int vIndex;
	int fIndex;
	int nIndex;

	char* file_name=fileName.GetBuffer(fileName.GetLength());
	FILE* pFile = fopen(file_name, "r");
	vertexNum=0;
	faceNum=0;

	while(!feof(pFile)&&!(fgetc(pFile)=='v'&&fgetc(pFile)==' '));
	vertexNum++;

	while(!feof(pFile))
	{   
		c=fgetc(pFile);
		if (c=='v'&&fgetc(pFile)!='n')
			vertexNum++;
		if (c=='f')
			faceNum++;
	}

	vertex=new MyVertex[vertexNum];
	normal=new Vector3d[vertexNum];
	face=new Face[faceNum];

	for (int i=0;i<vertexNum;i++)
	{
		vertex[i].index=i;
	}

	fseek(pFile,0,SEEK_SET);
	while(!feof(pFile)&&!(fgetc(pFile)=='v'&&fgetc(pFile)==' '));
	fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
	vertex[0].Pos=Vector3d(x,y,z);

	vIndex=1;
	fIndex=0;
	nIndex=0;


	while(!feof(pFile))
	{   
		c=fgetc(pFile);
		if (c=='v'){
			if (fgetc(pFile)!='n'){
				fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
				vertex[vIndex].Pos=Vector3d(x,y,z);
				vIndex++;
			}
			else{
				fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
				normal[nIndex]=Vector3d(x,y,z);
				nIndex++;
			}
		}
		if (c=='f'){
			fscanf(pFile,"%d//%d %d//%d %d//%d",&face[fIndex].first,&tmp,
				&face[fIndex].second,&tmp,&face[fIndex].third,&tmp);
			vertex[face[fIndex].first-1].AddNeighbor(face[fIndex].second-1);
			vertex[face[fIndex].first-1].AddNeighbor(face[fIndex].third-1);
			vertex[face[fIndex].second-1].AddNeighbor(face[fIndex].first-1);
			vertex[face[fIndex].second-1].AddNeighbor(face[fIndex].third-1);
			vertex[face[fIndex].third-1].AddNeighbor(face[fIndex].first-1);
			vertex[face[fIndex].third-1].AddNeighbor(face[fIndex].second-1);
			fIndex++;
		}
			
	}

	fclose(pFile);
}


void ObjModel::loadObjFromFileEx(CString fileName)
{
	char c;
	double x;
	double y;
	double z;
	int tmp;
	int vIndex;
	int fIndex;
	int nIndex;

	char* file_name=fileName.GetBuffer(fileName.GetLength());
	FILE* pFile = fopen(file_name, "r");
	vertexNum=0;
	faceNum=0;

	while(!feof(pFile)&&!(fgetc(pFile)=='v'&&fgetc(pFile)==' '));
	vertexNum++;

	while(!feof(pFile))
	{   
		c=fgetc(pFile);
		if (c=='v'&&fgetc(pFile)!='n')
			vertexNum++;
		if (c=='f')
			faceNum++;
	}

	vertex=new MyVertex[vertexNum];
	normal=new Vector3d[vertexNum];
	face=new Face[faceNum];

	for (int i=0;i<vertexNum;i++)
	{
		vertex[i].index=i;
	}

	fseek(pFile,0,SEEK_SET);
	while(!feof(pFile)&&!(fgetc(pFile)=='v'&&fgetc(pFile)==' '));
	fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
	vertex[0].Pos=Vector3d(x,y,z);

	vIndex=1;
	fIndex=0;
	nIndex=0;


	while(!feof(pFile))
	{   
		c=fgetc(pFile);
		if (c=='v'){
			if (fgetc(pFile)!='n'){
				fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
				vertex[vIndex].Pos=Vector3d(x,y,z);
				vIndex++;
			}
			else{
				fscanf(pFile,"%lf %lf %lf",&x,&y,&z);
				normal[nIndex]=Vector3d(x,y,z);
				nIndex++;
			}
		}
		if (c=='f'){
			fscanf(pFile,"%d %d %d",&face[fIndex].first,
				&face[fIndex].second,&face[fIndex].third);
			vertex[face[fIndex].first-1].AddNeighbor(face[fIndex].second-1);
			vertex[face[fIndex].first-1].AddNeighbor(face[fIndex].third-1);
			vertex[face[fIndex].second-1].AddNeighbor(face[fIndex].first-1);
			vertex[face[fIndex].second-1].AddNeighbor(face[fIndex].third-1);
			vertex[face[fIndex].third-1].AddNeighbor(face[fIndex].first-1);
			vertex[face[fIndex].third-1].AddNeighbor(face[fIndex].second-1);
			fIndex++;
		}

	}

	fclose(pFile);
}

void ObjModel::glDraw()
{
	GLfloat mat_diffuse[] = {0.3,0.2,0.5,0.5};
	GLfloat mat_specular[] = {1.0,1.0,1.0,0.5};
	GLfloat high_shininess[] = {50.0};
	srand(6);
	for (int i=0;i<64;i++)
	{
		color[i][0]=rand()/(double)(RAND_MAX+1);
		color[i][1]=rand()/(double)(RAND_MAX+1);
		color[i][2]=rand()/(double)(RAND_MAX+1);
		color[i][3]=0.5;
	}
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,high_shininess);
	glEnable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	int first;
	int second;
	int third;
	int cluster;
	for (int i=0;i<faceNum;i++)
	{
		first=face[i].first-1;
		second=face[i].second-1;
		third=face[i].third-1;

		/*if(vertex[second].cluster==vertex[second].cluster){
			cluster=vertex[second].cluster;
		}
		else*/
		cluster=vertex[first].cluster;

		if (cluster>=0)
		{
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,color[cluster]);
		}
		if (cluster==-1)
		{
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
		}
		glNormal3d(normal[first](0),normal[first](1),normal[first](2));
		glVertex3d(vertex[first].Pos(0),vertex[first].Pos(1),vertex[first].Pos(2));
		glNormal3d(normal[second](0),normal[second](1),normal[second](2));
		glVertex3d(vertex[second].Pos(0),vertex[second].Pos(1),vertex[second].Pos(2));
		glNormal3d(normal[third](0),normal[third](1),normal[third](2));
		glVertex3d(vertex[third].Pos(0),vertex[third].Pos(1),vertex[third].Pos(2));
	}
	glEnd();


	/*glDisable(GL_LIGHTING);
	glPointSize(10);
	glBegin(GL_POINTS);
		glVertex3d(vertex[7197].Pos(0),vertex[7197].Pos(1),vertex[7197].Pos(2));
	glEnd();*/

}

void ObjModel::glDraw(int cIndex)
{
	GLfloat mat_diffuse[] = {0.3,0.2,0.5,0.5};
	GLfloat mat_specular[] = {1.0,1.0,1.0,0.5};
	GLfloat high_shininess[] = {50.0};
	srand(6);
	for (int i=0;i<64;i++)
	{
		color[i][0]=rand()/(double)(RAND_MAX+1);
		color[i][1]=rand()/(double)(RAND_MAX+1);
		color[i][2]=rand()/(double)(RAND_MAX+1);
		color[i][3]=0.5;
	}
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,high_shininess);
	glEnable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	int first;
	int second;
	int third;
	int cluster;
	for (int i=0;i<faceNum;i++)
	{
		first=face[i].first-1;
		second=face[i].second-1;
		third=face[i].third-1;

		/*if(vertex[second].cluster==vertex[second].cluster){
			cluster=vertex[second].cluster;
		}
		else*/

		if (vertex[first].cluster==cIndex||vertex[second].cluster==cIndex||vertex[third].cluster==cIndex)
		{
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_specular);
		}
		else
		{
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
		}
		glNormal3d(normal[first](0),normal[first](1),normal[first](2));
		glVertex3d(vertex[first].Pos(0),vertex[first].Pos(1),vertex[first].Pos(2));
		glNormal3d(normal[second](0),normal[second](1),normal[second](2));
		glVertex3d(vertex[second].Pos(0),vertex[second].Pos(1),vertex[second].Pos(2));
		glNormal3d(normal[third](0),normal[third](1),normal[third](2));
		glVertex3d(vertex[third].Pos(0),vertex[third].Pos(1),vertex[third].Pos(2));
	}
	glEnd();


	/*glDisable(GL_LIGHTING);
	glPointSize(10);
	glBegin(GL_POINTS);
		glVertex3d(vertex[7197].Pos(0),vertex[7197].Pos(1),vertex[7197].Pos(2));
	glEnd();*/

}

void ObjModel::glDraw(double* V)
{
	GLfloat mat_diffuse[] = {0.3,0.2,0.5,0.5};
	GLfloat mat_specular[] = {1.0,1.0,1.0,0.5};
	GLfloat high_shininess[] = {50.0};
	srand(6);
	for (int i=0;i<64;i++)
	{
		color[i][0]=rand()/(double)(RAND_MAX+1);
		color[i][1]=rand()/(double)(RAND_MAX+1);
		color[i][2]=rand()/(double)(RAND_MAX+1);
		color[i][3]=0.5;
	}
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,high_shininess);
	glEnable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	int first;
	int second;
	int third;
	int cluster;
	for (int i=0;i<faceNum;i++)
	{
		first=face[i].first-1;
		second=face[i].second-1;
		third=face[i].third-1;

		/*if(vertex[second].cluster==vertex[second].cluster){
			cluster=vertex[second].cluster;
		}
		else*/

		double wC=V[first]+V[second]+V[third];
		wC*=2;
		if (wC>0)
		{
			GLfloat mat[3]={0,0,0};
			mat[0]=wC;
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat);
		}
		else
		{
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diffuse);
		}
		glNormal3d(normal[first](0),normal[first](1),normal[first](2));
		glVertex3d(vertex[first].Pos(0),vertex[first].Pos(1),vertex[first].Pos(2));
		glNormal3d(normal[second](0),normal[second](1),normal[second](2));
		glVertex3d(vertex[second].Pos(0),vertex[second].Pos(1),vertex[second].Pos(2));
		glNormal3d(normal[third](0),normal[third](1),normal[third](2));
		glVertex3d(vertex[third].Pos(0),vertex[third].Pos(1),vertex[third].Pos(2));
	}
	glEnd();


	/*glDisable(GL_LIGHTING);
	glPointSize(10);
	glBegin(GL_POINTS);
		glVertex3d(vertex[7197].Pos(0),vertex[7197].Pos(1),vertex[7197].Pos(2));
	glEnd();*/

}

double ObjModel::calDist(ObjModel& refObj)
{
	double dist=0;

	for (int i=0;i<vertexNum;i++)
	{
		dist+=(vertex[i].Pos-refObj.vertex[i].Pos).squaredNorm();
	}
	return sqrt(dist/vertexNum);
}

void ObjModel::writeToFile(CString fileName)
{
	char *fName =(LPSTR)(LPCTSTR)fileName;
	FILE* fp=fopen(fName,"w+");

	for (int i=0;i<vertexNum;i++)
	{
		Vector3d& p=vertex[i].Pos;
		fprintf(fp,"v %lf %lf %lf\n",p(0),p(1),p(2));
	}

	for (int i=0;i<faceNum;i++)
	{
		fprintf(fp,"f %d %d %d\n",face[i].first,face[i].second,face[i].third);
	}

	fclose(fp);
}