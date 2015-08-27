#include "stdafx.h"
#include "MotionCluster.h"
#include <iostream>
#include <algorithm>
#include "Eigen/SVD"
int myCompare(DistPair& p1,DistPair& p2)
{
	return p1.dist<p2.dist;
}

int myCompareEx(DistPair& p1,DistPair& p2)
{
	return p1.dist>p2.dist;
}

MotionCluster::MotionCluster(void)
{
	weight=NULL;
	joint=NULL;
	BC=NULL;
	L=NULL;
	paraW=1e-2;
	paraL=1;
}


MotionCluster::~MotionCluster(void)
{
	if (weight!=NULL)
	{
		for (int i=0;i<vertexNum;i++)
		{
			delete[] weight[i];
		}
		delete[] weight;
	}
	if (joint!=NULL)
	{
		for (int i=0;i<boneNum;i++)
		{
			delete[] joint[i];
		}
		delete[] joint;
	}
	if (L!=NULL)
	{
		for (int i=0;i<vertexNum;i++)
		{
			delete[] L[i];
		}
		delete[] L;
	}
	if (BC!=NULL)
	{
		for(int i=0;i<vertexNum;i++)
		{
			delete[] BC[i];
		}
		delete[] BC;
	}

}

void MotionCluster::copyHMatrix3D(HMatrix& hmat,Matrix3d& mat)
{
	memset(hmat[0],0,4*sizeof(float));
	memset(hmat[1],0,4*sizeof(float));
	memset(hmat[2],0,4*sizeof(float));
	memset(hmat[3],0,4*sizeof(float));
	for (int i=0;i<3;i++)
	{
		for (int j = 0; j < 3; j++)
		{
			hmat[i][j]=mat(i,j);
		}
	}
	hmat[3][3]=1;
}

void MotionCluster::copyFromHMatrix3D(HMatrix& hmat,MatrixXd& mat)
{
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			mat(i,j)=hmat[i][j];
		}
	}
}

Vector3d MotionCluster::transformPoint(Vector3d& point,MatrixXd& transform)
{
	Vector4d tmp;
	tmp.head(3)=point;
	tmp(3)=1;
	tmp=transform*tmp;
	return tmp.head(3);

}

void MotionCluster::loadObjs(CString& strPath)
{
	CFileFind fileFinder;
	strPath+="*.*";
	bool bWorking=fileFinder.FindFile(strPath);
	while (bWorking)
	{
		bWorking=fileFinder.FindNextFile();
		if( !fileFinder.IsDirectory() && !fileFinder.IsDots() )
		{
			CString dataPath=fileFinder.GetFilePath();
			Objs.push_back(ObjModel(dataPath));
		}
		
	}

	frameNum=Objs.size();
	vertexNum=Objs.at(0).vertexNum;
}


void MotionCluster::generateClusters()
{
	Cluster C;
	for (int i=0;i<vertexNum;i++)
	{
		C.vIndex.push_back(i);
	}
	calculateTransformation(C);
	/*MatrixXd trans=C.transform.at(2);
	for (int i = 0; i < vertexNum; i++)
	{
		Objs.at(0).vertex[i].Pos=transformPoint(Objs.at(0).vertex[i].Pos,trans);
	}*/
	
	clusterSet.push_back(C);

	double initialError=0;
	double postError=0;

	initialError=calClusterError(C);


	for (int i=0;i<6;i++)
	{
		vector<Cluster> tmpSet;
		for (int j=0;j<clusterSet.size();j++)
		{
			Cluster& tempC=clusterSet.at(j);
			if (calClusterError(tempC)>0.002)
			{
				splitCluster(tempC,tmpSet);
			}
			else{
				tmpSet.push_back(tempC);
			}		
		}
		clusterSet=tmpSet;
	}

	//assign non-bone points
	int* V=new int[vertexNum];

	for (int i=0;i<vertexNum;i++)
	{
		V[i]=-1;
	}
	for (int i=0;i<clusterSet.size();i++)
	{
		vector<int>& vIndex=clusterSet.at(i).vIndex;
		for (int j=0;j<vIndex.size();j++)
		{
			V[vIndex.at(j)]=i;
		}
	}
	for (int i=0;i<vertexNum;i++)
	{
		if (V[i]==-1)
		{
			int* B=new int[clusterSet.size()];
			for (int j = 0; j < clusterSet.size(); j++)
			{
				B[j]=0;
			}
			vector<int> neighbor=Objs.at(0).vertex[i].Neighbor;
			for (int j=0;j<neighbor.size();j++)
			{
				B[V[neighbor.at(j)]]=1;
			}
			int bIndex;
			double minErr=99999;
			for (int j=0;j<clusterSet.size();j++)
			{
				if (B[j]==0)
					continue;
				double err=calReconstrError(i,clusterSet.at(j).transform);
				if (err<minErr)
				{
					minErr=err;
					bIndex=j;
				}
			}
			clusterSet.at(bIndex).vIndex.push_back(i);
		}
	}
	for (int i=0;i<clusterSet.size();i++)
	{
		calculateTransformation(clusterSet.at(i));
	}
	delete[] V;

	//end
	boneNum=clusterSet.size();
	printf("clusterSetSize:%d\n",boneNum);
	
	int total=0;
	for (int i=0;i<clusterSet.size();i++)
	{
		cout<<i<<":"<<clusterSet.at(i).vIndex.size()<<endl;
		total+=clusterSet.at(i).vIndex.size();
	}
	cout<<total<<endl;

	for (int i=0;i<clusterSet.size();i++)
	{
		//if (i!=0&&i!=1)
			//continue;
		Cluster& tempC=clusterSet.at(i);
		double err=calClusterError(tempC);
		postError+=err;
		//printf("%d %lf\n",tempC.vIndex.size(),err/tempC.vIndex.size());
		for (int j=0;j<tempC.vIndex.size();j++)
		{
			int index=tempC.vIndex.at(j);
			Objs.at(0).vertex[index].cluster=i;
			Objs.at(1).vertex[index].cluster=i;
		}
	}
	postError/=clusterSet.size();


	//printf("%lf %lf",initialError,postError);
}

void MotionCluster::calculateTransformation(Cluster& C)
{
	C.transform.clear();
	ObjModel& restModel=Objs.at(0);
	vector<int> vIndex=C.vIndex;
	int cSize=vIndex.size();

	for (int i=1;i<frameNum;i++)
	{
		ObjModel& motionModel=Objs.at(i);
		Vector3d rCenter(0,0,0);
		Vector3d mCenter(0,0,0);
		Matrix3d mat;
		mat.setZero();
		for (int j=0;j<cSize;j++)
		{
			int index=vIndex.at(j);
			rCenter+=restModel.vertex[index].Pos;
			mCenter+=motionModel.vertex[index].Pos;
		}
		rCenter/=cSize;
		mCenter/=cSize;

		for (int j=0;j<cSize;j++)
		{
			int index=vIndex.at(j);
			Vector3d r=restModel.vertex[index].Pos;
			Vector3d m=motionModel.vertex[index].Pos;
			mat+=(m-mCenter)*(r-rCenter).transpose();
		}
		HMatrix hmat;
		HMatrix	Q,S;
		copyHMatrix3D(hmat,mat);
		polar_decomp(hmat,Q,S);
		
		MatrixXd trans(3,4);
		MatrixXd trans3D(3,3);
		Vector3d translation;
		copyFromHMatrix3D(Q,trans3D);
		copyFromHMatrix3D(Q,trans);
		translation=mCenter-trans3D*rCenter;
		trans.col(3)=translation;
		C.transform.push_back(trans);
	}
}

void MotionCluster::splitCluster(const Cluster& C,vector<Cluster>& cSet)
{
	vector<int> vIndex=C.vIndex;
	int cSize=vIndex.size();
	ObjModel& restModel=Objs.at(0);
	Vector3d center(0,0,0);

	//caculate O;
	for (int i=0;i<cSize;i++)
	{
		int index=vIndex.at(i);
		center+=restModel.vertex[index].Pos;
	}
	center/=cSize;

	double sMax=-1;
	int sIndex=0;;

	for (int i=0;i<cSize;i++)
	{
		int index=vIndex.at(i);
		double sTemp;
		double dist;
		dist=(restModel.vertex[index].Pos-center).norm();
		sTemp=dist*calReconstrError(index,C.transform);
		if (sTemp>sMax)
		{
			sMax=sTemp;
			sIndex=index;
		}
	}

	//printf("%d\n",sIndex);

	vector<DistPair> distPairs;
	for (int i=0;i<cSize;i++)
	{
		int index=vIndex.at(i);
		DistPair distPair;
		distPair.index=index;
		distPair.dist=(restModel.vertex[index].Pos-restModel.vertex[sIndex].Pos).norm();
		distPairs.push_back(distPair);
	}
	sort(distPairs.begin(),distPairs.end(),myCompare);

	Cluster A,B;
	for (int i=0;i<distPairs.size();i++)
	{
		//printf("%d %lf\n",distPairs.at(i).index,distPairs.at(i).dist);
		if (i<cSize/2)
		{
			A.vIndex.push_back(distPairs.at(i).index);
		}
		else{
			B.vIndex.push_back(distPairs.at(i).index);
		}
	}

	for (int i=0;i<10;i++)
	{
		calculateTransformation(A);
		calculateTransformation(B);
		A.vIndex.clear();
		B.vIndex.clear();
		for (int j=0;j<cSize;j++)
		{
			int index=vIndex.at(j);
			double reconsErrorA=calReconstrError(index,A.transform);
			double reconsErrorB=calReconstrError(index,B.transform);
			if (reconsErrorA<=reconsErrorB)
				A.vIndex.push_back(index);
			else
				B.vIndex.push_back(index);
		}
	}
	
	if (A.vIndex.size()>vertexNum*0.001&&B.vIndex.size()>vertexNum*0.001)
	{
		vector<Cluster> tmpSet;
		bool res=splitPatch(A,tmpSet)&&splitPatch(B,tmpSet);
		if (res)
		{
			for (int i=0;i<tmpSet.size();i++)
			{
				cSet.push_back(tmpSet.at(i));
			}
		}
		else
			cSet.push_back(C);
	}
	else{
		cSet.push_back(C);
	}

}

bool MotionCluster::splitPatch(const Cluster& C,vector<Cluster>& cSet)
{
	vector<Cluster> tmpSet;
	int* V=new int[vertexNum];
	for (int i=0;i<vertexNum;i++)
	{
		V[i]=-1;
	}
	for (int i=0;i<C.vIndex.size();i++)
	{
		int index=C.vIndex.at(i);
		V[index]=0;
	}
	int bIndex=C.vIndex.at(0);
	while(1)
	{
		Cluster A;
		ConnectedSearch(bIndex,V);
		bIndex=-1;
		for (int i=0;i<vertexNum;i++)
		{
			if (V[i]==0&&bIndex==-1){
				bIndex=i;
			}
			if (V[i]==1)
			{
				A.vIndex.push_back(i);
				V[i]=-1;
			}
		}
		tmpSet.push_back(A);
		if (bIndex==-1)
			break;
	}	
	
	printf("%d\n",tmpSet.size());

	if (tmpSet.size()==1)
	{
		cSet.push_back(C);
	}
	else if (tmpSet.size()==2)
	{
		for (int i=0;i<2;i++)
		{
			Cluster& T=tmpSet.at(i);
			if (T.vIndex.size()>vertexNum*0.001)
			{
				calculateTransformation(T);
				cSet.push_back(T);
			}
		}
	}
	else
	{
		bool res=true;
		for (int i=0;i<tmpSet.size();i++)
		{
			Cluster& T=tmpSet.at(i);
			printf("%lf\n",T.vIndex.size()/(double)C.vIndex.size());
			//if (T.vIndex.size()/(double)C.vIndex.size()>0.7)
			//{
			if (T.vIndex.size()>vertexNum*0.001)
			{
				calculateTransformation(T);
				cSet.push_back(T);
			}
			//	res=true;
			//	break;
			//}
		}
		if (res)
		{
			delete[] V;
			return true;
		}
		else{
			delete[] V;
			return false;
		}
	}

	delete[] V;
	return true;

}

void MotionCluster::ConnectedSearch(int index,int* V)
{
	V[index]=1;
	vector<int> neighbor=Objs.at(0).vertex[index].Neighbor;
	for (int i=0;i<neighbor.size();i++)
	{
		int nIndex=neighbor.at(i);
		if (V[nIndex]==0)
		{
			ConnectedSearch(nIndex,V);
		}
	}
}

double MotionCluster::calReconstrError(int index,const vector<MatrixXd>& transform)
{
	double reconsError=0;
	Vector3d restPoint=Objs.at(0).vertex[index].Pos;

	for (int i=1;i<frameNum;i++)
	{
		Vector3d motionPoint=Objs.at(i).vertex[index].Pos;
		MatrixXd trans=transform.at(i-1);
		Vector3d reconsPoint=transformPoint(restPoint,trans);
		reconsError+=(motionPoint-reconsPoint).squaredNorm();
	}
	reconsError=sqrt(reconsError/(frameNum-1));
	return reconsError;
}

double MotionCluster::calClusterError(const Cluster& C)
{
	double Error=0;
	for (int j=0;j<C.vIndex.size();j++)
	{
		int index=C.vIndex.at(j);
		Error+=calReconstrError(index,C.transform);
	}
	return Error/C.vIndex.size();
}

double MotionCluster::calTotalError()
{
	double totalError=0;
	for (int i=0;i<vertexNum;i++)
	{
		Vector3d p=Objs.at(0).vertex[i].Pos;
		for (int t=0;t<frameNum-1;t++)
		{
			Vector3d v=Objs.at(t+1).vertex[i].Pos;
			for (int j=0;j<boneNum;j++)
			{
				if (weight[i][j]!=0)
				{
					v-=weight[i][j]*transformPoint(p,clusterSet.at(j).transform.at(t));
				}
			}
			totalError+=v.squaredNorm();
		}
	}
	return totalError;
}

void MotionCluster::pruneBones(vector<int> boneIndex)
{
	int* V=new int[boneNum];
	vector<Cluster> newClusterSet;
	for (int i=0;i<boneNum;i++)
	{
		V[i]=1;
	}
	for (int i=0;i<boneIndex.size();i++)
	{
		V[boneIndex.at(i)]=0;
	}
	for (int i=0;i<boneNum;i++)
	{
		if (V[i]==1)
		{
			newClusterSet.push_back(clusterSet.at(i));
		}
	}
	clusterSet=newClusterSet;

	double** newWeight;
	newWeight=new double*[vertexNum];
	for (int i=0;i<vertexNum;i++)
	{
		newWeight[i]=new double[boneNum-boneIndex.size()];
	}
	int p=0;
	for (int i=0;i<boneNum;i++)
	{
		if (V[i]==0)
			continue;
		for (int j=0;j<vertexNum;j++)
		{
			newWeight[j][p]=weight[j][i];
		}
		p++;
	}
	
	for (int i=0;i<vertexNum;i++)
	{
		delete[] weight[i];
	}
	delete[] weight;

	weight=newWeight;
	boneNum-=boneIndex.size();

	delete[] V;

	if (BC!=NULL)
	{
		for (int i=0;i<vertexNum;i++)
		{
			delete BC[i];
		}
		delete[] BC;
		BC=NULL;
	}
}

void MotionCluster::SSDRstep()
{
	weight=new double*[vertexNum];
	for (int i=0;i<vertexNum;i++)
	{
		weight[i]=new double[boneNum];
		memset(weight[i],0,boneNum*sizeof(double));
	}

	for (int i=0;i<boneNum;i++)
	{
		Cluster& C=clusterSet.at(i);
		for (int j=0;j<C.vIndex.size();j++)
		{
			int index=C.vIndex.at(j);
			weight[index][i]=1;
		}
	}

	cout<<endl<<"initialError:"<<calTotalError()<<endl;
	int nz=1;
	/*for (int i=0;i<4;i++)
	{
		updateWeight(4,false);

		cout<<"prevError:"<<calTotalError()<<endl;
		updateTransform();
		cout<<"afterError:"<<calTotalError()<<endl;

	}*/

	for (int i=0;i<3;i++)
	{
		updateWeight(4,false);
		initBoneWeightConstrain(true);
		updateWeight(4,true);
		cout<<"prevError:"<<calTotalError()<<endl;
		updateTransform();
		cout<<"afterError:"<<calTotalError()<<endl;

	}


	bool res1=updateClusters(true);
	/*while (res1)
	{
		for (int i=0;i<1;i++)
		{
			updateWeight(4,false);
			initBoneWeightConstrain();
			updateWeight(4,true);
			//cout<<"prevError:"<<calTotalError()<<endl;
			updateTransform();
			//cout<<"afterError:"<<calTotalError()<<endl;

		}
		res1=updateClusters(true);
	}*/

}

bool MotionCluster::prune(double threshold,bool test)
{
	double *w;
	w=new double[boneNum];

	double maxW=-1;
	for (int i=0;i<boneNum;i++)
	{
		w[i]=0;
		for (int j=0;j<vertexNum;j++)
		{
			w[i]+=weight[j][i]*weight[j][i];
		}
		if (w[i]>maxW)
		{
			maxW=w[i];
		}
	}

	vector<int> pruneIndex;

	for (int i=0;i<boneNum;i++)
	{
		if (!test)
		{
			double nW=99999;
			for (int j=0;j<MSTedgeSet.size();j++)
			{
				EdgeNode& edge=MSTedgeSet.at(j);
				if (edge.v1==i)
				{
					if (w[edge.v2]<nW)
					{
						nW=w[edge.v2];
					}
				}
				if (edge.v2==i)
				{
					if (w[edge.v1]<nW)
					{
						nW=w[edge.v1];
					}
				}
			}

			if (w[i]>nW*0.8)
			{
				continue;
			}
		}

		if (w[i]<maxW*threshold)
		{
			pruneIndex.push_back(i);
		}
	}

	/*double maxW=-1;
	for (int i=0;i<boneNum;i++)
	{
		w[i]=0;
		for (int j=0;j<vertexNum;j++)
		{
			w[i]+=weight[j][i]*weight[j][i];
		}
		if (w[i]>maxW)
		{
			maxW=w[i];
		}
	}

	vector<int> pruneIndex;
	for (int i=0;i<boneNum;i++)
	{
		if (w[i]<maxW*0.01)
		{
			pruneIndex.push_back(i);
		}
	}*/

	delete[] w;

	if (pruneIndex.size()>0)
	{
		pruneBones(pruneIndex);
		return true;
	}
	else{
		return false;
	}
	
}

void MotionCluster::updateWeight(int nzWeightNum,bool BCconstructed)
{
	for (int i=0;i<vertexNum;i++)
	{
		updateVertexWeight(i,nzWeightNum,BCconstructed);
		//cout<<i<<endl;
	}
}

void MotionCluster::updateVertexWeight(int vIndex,int nzWeightNum,bool BCconstructed)
{
	MatrixXd m_A=initA(vIndex);
	VectorXd m_b=initb(vIndex);
	alglib::real_1d_array lb, ub;
	initBound(lb,ub,boneNum);

	alglib::real_2d_array cs;
	alglib::integer_1d_array ct;
	initConstrain(cs,ct,boneNum);
	alglib::real_1d_array sp;
	sp.setlength(boneNum);
	for (int i=0;i<boneNum;i++)
	{
		sp[i]=weight[vIndex][i];
	}
	VectorXd w=solveLeastSquare(m_A,m_b,lb,ub,cs,ct,sp);

	vector<DistPair> distPairs;
	if (BCconstructed)
	{
		for (int i=0;i<w.size();i++)
		{
			if (BC[vIndex][i]==1){
				DistPair dP;
				dP.dist=calEffect(vIndex,i,w);;
				dP.index=i;
				distPairs.push_back(dP);
			}	
		}
	}
	else{
		for (int i=0;i<w.size();i++)
		{
			if (w(i)>0){
				DistPair dP;
				dP.dist=calEffect(vIndex,i,w);;
				dP.index=i;
				distPairs.push_back(dP);
			}	
		}
	}

	sort(distPairs.begin(),distPairs.end(),myCompareEx);
	vector<int> boneIndex;
	for (int i=0;i<distPairs.size()&&i<nzWeightNum;i++)
	{
		boneIndex.push_back(distPairs.at(i).index);
	}

	m_A=initA_ex(vIndex,boneIndex);
	initBound(lb,ub,boneIndex.size());
	initConstrain(cs,ct,boneIndex.size());
	sp.setlength(boneIndex.size());
	for (int i=0;i<boneIndex.size();i++)
	{
		sp[i]=w(boneIndex.at(i));
	}
	w=solveLeastSquare(m_A,m_b,lb,ub,cs,ct,sp);

	for (int i=0;i<boneNum;i++)
	{
		weight[vIndex][i]=0;
	}
	for (int i=0;i<boneIndex.size();i++)
	{
		weight[vIndex][boneIndex.at(i)]=w(i);
	}
}

double MotionCluster::calEffect(int vIndex,int bIndex,VectorXd w)
{
	double Error=0;
	Vector3d p=Objs.at(0).vertex[vIndex].Pos;
	for (int t=0;t<frameNum-1;t++)
	{
		Vector3d v=Objs.at(t+1).vertex[vIndex].Pos;
		for (int i=0;i<boneNum;i++)
		{
			if (i==bIndex)
			{
				continue;
			}
			v-=w(i)*transformPoint(p,clusterSet.at(i).transform.at(t));
		}
		
		Error+=v.squaredNorm();
	}

	return Error;
}

double MotionCluster::calResidueEffect(int vIndex,int bIndex,double w)
{
	double Error=0;
	Vector3d p=Objs.at(0).vertex[vIndex].Pos;
	for (int t=0;t<frameNum-1;t++)
	{
		Vector3d v=Objs.at(t+1).vertex[vIndex].Pos;
		v-=w*transformPoint(p,clusterSet.at(bIndex).transform.at(t));
		Error+=v.squaredNorm();
	}

	return Error;
}

MatrixXd MotionCluster::initA(int vIndex)
{
	MatrixXd A(3*(frameNum-1),boneNum);
	for (int i=0;i<frameNum-1;i++)
	{
		for (int j=0;j<boneNum;j++)
		{
			Vector3d p=Objs.at(0).vertex[vIndex].Pos;
			p=transformPoint(p,clusterSet.at(j).transform.at(i));
			A(3*i,j)=p(0);
			A(3*i+1,j)=p(1);
			A(3*i+2,j)=p(2);
		}
	}
	return A;
}

MatrixXd MotionCluster::initA_ex(int vIndex,vector<int> boneIndex)
{
	MatrixXd A(3*(frameNum-1),boneIndex.size());
	for (int i=0;i<frameNum-1;i++)
	{
		for (int j=0;j<boneIndex.size();j++)
		{
			int index=boneIndex.at(j);
			Vector3d p=Objs.at(0).vertex[vIndex].Pos;
			p=transformPoint(p,clusterSet.at(index).transform.at(i));
			A(3*i,j)=p(0);
			A(3*i+1,j)=p(1);
			A(3*i+2,j)=p(2);
		}
	}
	return A;
}

VectorXd MotionCluster::initb(int vIndex)
{
	VectorXd b(3*(frameNum-1));
	for (int i=0;i<frameNum-1;i++)
	{
		Vector3d v=Objs.at(i+1).vertex[vIndex].Pos;
		b(3*i)=-v(0);
		b(3*i+1)=-v(1);
		b(3*i+2)=-v(2);
	}
	return b;
}

void MotionCluster::initBound(alglib::real_1d_array& lb,alglib::real_1d_array& ub,int size)
{
	lb.setlength(size);
	ub.setlength(size);
	for (int i=0;i<size;i++)
	{
		lb[i]=0;
		ub[i]=alglib::fp_posinf;
	}
}

void MotionCluster::initConstrain(alglib::real_2d_array& cs,alglib::integer_1d_array& ct,int size)
{
	cs.setlength(1,size+1);
	ct.setlength(1);
	for (int i=0;i<size+1;i++)
	{
		cs[0][i]=1;
	}
	ct[0]=0;
}

alglib::real_2d_array MotionCluster::transformMatrix(MatrixXd& A)
{
	alglib::real_2d_array B;
	int rowNum=A.rows();
	int colNum=A.cols();
	B.setlength(rowNum,colNum);
	for (int i=0;i<rowNum;i++)
	{
		for (int j=0;j<colNum;j++)
		{
			B[i][j]=A(i,j);
		}
	}
	return B;
}

alglib::real_1d_array MotionCluster::transformVector(VectorXd& A)
{
	alglib::real_1d_array B;
	int length=A.size();
	B.setlength(length);
	for (int i=0;i<length;i++)
	{
		B[i]=A(i);
	}
	return B;
}

VectorXd MotionCluster::solveLeastSquare(MatrixXd& A,VectorXd& b,alglib::real_1d_array& lb,alglib::real_1d_array& ub 
									 ,alglib::real_2d_array& cs,alglib::integer_1d_array& ct,alglib::real_1d_array& sp)
{
	MatrixXd ATA=A.transpose()*A;
	VectorXd ATb=A.transpose()*b;
	alglib::real_2d_array m_ATA;
	alglib::real_1d_array m_ATb; 
	m_ATA=transformMatrix(ATA);
	m_ATb=transformVector(ATb);

	alglib::minqpstate m_qp_optimizer;

	alglib::minqpcreate(A.cols(), m_qp_optimizer);
	//ATA
	alglib::minqpsetquadraticterm(m_qp_optimizer,m_ATA);
	//ATw
	alglib::minqpsetlinearterm(m_qp_optimizer,m_ATb);
	//init qp solver
	alglib::minqpsetalgobleic(m_qp_optimizer,0,0,0,0);

	alglib::minqpsetbc(m_qp_optimizer, lb, ub);
	alglib::minqpsetlc(m_qp_optimizer,cs,ct);
	alglib::minqpsetstartingpoint(m_qp_optimizer,sp);

	alglib::minqpoptimize(m_qp_optimizer);

	alglib::real_1d_array x;
	alglib::minqpreport rep;
	alglib::minqpresults(m_qp_optimizer, x, rep);
	VectorXd w(x.length());
	for (int i=0;i<x.length();i++)
	{
		w(i)=x[i];
	}
	return w;
}

void MotionCluster::updateTransform()
{
	for (int i=0;i<boneNum;i++)
	{
		updateBoneTransForm(i);
	}
}

void MotionCluster::updateBoneTransForm(int bIndex)
{
	Cluster& C=clusterSet.at(bIndex);
	//cout<<"Old "<<bIndex<<":"<<calClusterError(C)<<endl;


	Vector3d p(0,0,0);
	double w=0;
	for (int i=0;i<vertexNum;i++)
	{
		double tmpW=weight[i][bIndex];
		p+=tmpW*tmpW*Objs.at(0).vertex[i].Pos;
		w+=tmpW*tmpW;
	}
	p/=w;
	//printf("%lf",w);

	Vector3d* qI=new Vector3d[vertexNum];

	for (int t=0;t<C.transform.size();t++)
	{
		MatrixXd trans=C.transform.at(t);
		MatrixXd P(3,vertexNum);
		MatrixXd Q(3,vertexNum);

		Vector3d q(0,0,0);

		for (int i=0;i<vertexNum;i++)
		{
			qI[i]=Objs.at(t+1).vertex[i].Pos;
			Vector3d pI=Objs.at(0).vertex[i].Pos;
			for (int j=0;j<boneNum;j++)
			{
				if (j!=bIndex&&weight[i][j]>0)
				{
					qI[i]-=weight[i][j]*transformPoint(pI,clusterSet.at(j).transform.at(t));
				}
			}
			q+=weight[i][bIndex]*qI[i];
		}
		q/=w;

		for (int i=0;i<vertexNum;i++)
		{
			Vector3d pI=Objs.at(0).vertex[i].Pos-p;
			P.col(i)=pI*weight[i][bIndex];
			qI[i]-=weight[i][bIndex]*q;
			Q.col(i)=qI[i];
			//cout<<pI;
		}

		MatrixXd M=P*Q.transpose();
		JacobiSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
		MatrixXd u=svd.matrixU();
		MatrixXd v=svd.matrixV();
		MatrixXd R=v*u.transpose();
		VectorXd T=q-R*p;
		MatrixXd new_trans(3,4);
		new_trans.leftCols(3)=R;
		new_trans.col(3)=T;
		//cout<<trans<<endl;
		//cout<<new_trans<<endl;
		C.transform.at(t)=new_trans;

	}
	//cout<<"New "<<bIndex<<":"<<calClusterError(C)<<endl;
	delete[] qI;
}

bool MotionCluster::updateClusters(bool update)
{
	bool res=false;

	if (update)
	{
		for (int i=0;i<clusterSet.size();i++)
		{
			clusterSet.at(i).vIndex.clear();
		}
		for (int i=0;i<vertexNum;i++)
		{
			double minR=99999;
			int bIndex;
			for (int j=0;j<boneNum;j++)
			{
				if (weight[i][j]==0)
					continue;

				double residue=calResidueEffect(i,j,weight[i][j]);
				if (residue<minR)
				{
					minR=residue;
					bIndex=j;
				}
			}
			clusterSet.at(bIndex).vIndex.push_back(i);
		}
	}

	vector<int> pruneIndex;
	for (int i=0;i<clusterSet.size();i++)
	{
		//cout<<i<<":"<<clusterSet.at(i).vIndex.size()<<endl;
		if (clusterSet.at(i).vIndex.size()<vertexNum*0.001)
		{
			pruneIndex.push_back(i);
		}
	}
	if (pruneIndex.size()>0){
		pruneBones(pruneIndex);
		res=true;
	}

	if (update&&res)
	{
		for (int i=0;i<clusterSet.size();i++)
		{
			clusterSet.at(i).vIndex.clear();
		}
		for (int i=0;i<vertexNum;i++)
		{
			double minR=99999;
			int bIndex;
			for (int j=0;j<boneNum;j++)
			{
				if (weight[i][j]==0)
					continue;

				double residue=calResidueEffect(i,j,weight[i][j]);
				if (residue<minR)
				{
					minR=residue;
					bIndex=j;
				}
			}
			clusterSet.at(bIndex).vIndex.push_back(i);
		}
	}

	double w;
	for (int i=0;i<boneNum;i++)
	{
		Cluster& C=clusterSet.at(i);
		for (int j=0;j<C.vIndex.size();j++)
		{
			int index=C.vIndex.at(j);
			Objs.at(0).vertex[index].cluster=i;
		}
	}
	cout<<"updateCSize:"<<clusterSet.size()<<endl;
	return res;
}

void MotionCluster::TopologyReconstr()
{
	if (joint!=NULL)
	{
		for (int i=0;i<boneNum;i++)
		{
			delete[] joint[i];
		}
		delete[] joint;
	}
	joint=new BoneJoint*[boneNum];
	for (int i=0;i<boneNum;i++)
	{
		joint[i]=new BoneJoint[boneNum];
	}

	//calJointCenterAndWeight(14,15);

	MST mst(boneNum);

	for (int i=0;i<boneNum-1;i++)
	{
		for (int j=i+1;j<boneNum;j++)
		{
			calJointCenterAndWeight(i,j);
			EdgeNode edge;
			edge.v1=i;
			edge.v2=j;
			edge.value=(joint[i][j]).weight;
			mst.Push(edge);
		}
	}
	mst.Kruskal();
	MSTedgeSet=mst.getEdgeSet();
	
}

void MotionCluster::calJoint()
{
	for (int i=0;i<boneNum-1;i++)
	{
		for (int j=i+1;j<boneNum;j++)
		{
			calJointCenterAndWeight(i,j);
		}
	}
}

void MotionCluster::calJointCenterAndWeight(int indexA,int indexB)
{
	BoneJoint& Joint=joint[indexA][indexB];
	Cluster& A=clusterSet.at(indexA);
	Cluster& B=clusterSet.at(indexB);
	int sizeA=A.vIndex.size();
	int sizeB=B.vIndex.size();

	MatrixXd AeqA(3*(frameNum-1),3);
	MatrixXd AeqB(3*(frameNum-1),3);
	VectorXd pA(3*(frameNum-1));
	VectorXd pB(3*(frameNum-1));

	for (int i=0;i<frameNum-1;i++)
	{
		MatrixXd& transA=A.transform.at(i);
		MatrixXd& transB=B.transform.at(i);
		AeqA.middleRows(3*i,3)=transA.leftCols(3);
		AeqB.middleRows(3*i,3)=transB.leftCols(3);
		pA.middleRows(3*i,3)=-transA.col(3);
		pB.middleRows(3*i,3)=-transB.col(3);
	}

	MatrixXd Aeq=AeqA-AeqB;
	VectorXd p=pA-pB;

	vector<Vector3d> centers=calBoundaryCenter(A,B);
	
	MatrixXd newAeq(6*(frameNum-1),3);
	VectorXd newP(6*(frameNum-1));

	newAeq.topRows(3*(frameNum-1))=Aeq;
	newP.topRows(3*(frameNum-1))=p;

	MatrixXd AeqEx=(AeqA+AeqB)/2;
	VectorXd pEx=(pA+pB)/2;
	for (int i=0;i<frameNum-1;i++)
	{
		Vector3d center=centers.at(i);
		//cout<<center<<endl;
		pEx.middleRows(3*i,3)+=center;
	}
	
	newAeq.bottomRows(3*(frameNum-1))=AeqEx*sqrt(frameNum*0.01);
	newP.bottomRows(3*(frameNum-1))=pEx*sqrt(frameNum*0.01);
	//cout<<Aeq<<endl;
	//cout<<p<<endl;
	VectorXd X=newAeq.jacobiSvd(ComputeThinU | ComputeThinV).solve(newP);
	//cout<<X<<endl;
	//cout<<"error:"<<(newAeq*X-newP).squaredNorm()<<endl;
	Center=X;
	Joint.center=X;
	
	double w=0;
	for (int i=0;i<vertexNum;i++)
	{
		w+=weight[i][indexA]*weight[i][indexB];
	}
	if (w==0)
	{
		Joint.weight=99999;
	}
	else{
		Joint.weight=(Aeq*Joint.center-p).squaredNorm()/w;
	}
	//cout<<indexA<<" "<<indexB<<":"<<Joint.weight<<endl;

}

vector<Vector3d> MotionCluster::calBoundaryCenter(Cluster& A,Cluster& B)
{
	int* V=new int[vertexNum];
	for (int i=0;i<vertexNum;i++)
	{
		V[i]=-1;
	}
	for (int i=0;i<A.vIndex.size();i++)
	{
		int index=A.vIndex.at(i);
		V[index]=0;
	}
	for (int i=0;i<B.vIndex.size();i++)
	{
		int index=B.vIndex.at(i);
		V[index]=1;
	}

	int startP=A.vIndex.at(0);
	while (1)
	{
		BoundarySearch(startP,V);
		startP=-1;
		for (int i=0;i<vertexNum;i++)
		{
			if (V[i]==0)
			{
				startP=i;
			}
		}
		if (startP==-1)
			break;
		//cout<<"test"<<endl;
	}
	

	vector<int> boundIndex;
	for (int i=0;i<vertexNum;i++)
	{
		if (V[i]==2)
		{
			boundIndex.push_back(i);
		}
	}

	vector<Vector3d> centers;
	if (boundIndex.size()==0)
	{
		for (int t=0;t<frameNum-1;t++)
		{
			Vector3d center(0,0,0);
			for (int i=0;i<A.vIndex.size();i++)
			{
				int index=A.vIndex.at(i);
				center+=Objs.at(t+1).vertex[index].Pos;
			}	
			for (int i=0;i<B.vIndex.size();i++)
			{
				int index=B.vIndex.at(i);
				center+=Objs.at(t+1).vertex[index].Pos;
			}	
			centers.push_back(center/(A.vIndex.size()+B.vIndex.size()));
		}
	}
	else
	{
		for (int t=0;t<frameNum-1;t++)
		{
			Vector3d center(0,0,0);
			for (int i=0;i<boundIndex.size();i++)
			{
				center+=Objs.at(t+1).vertex[boundIndex.at(i)].Pos;
			}	
			centers.push_back(center/boundIndex.size());
		}
	}

	return centers;
}

void MotionCluster::BoundarySearch(int index,int* V)
{
	V[index]=-1;
	vector<int> neighbor=Objs.at(0).vertex[index].Neighbor;
	for (int i=0;i<neighbor.size();i++)
	{
		int nIndex=neighbor.at(i);
		if (V[nIndex]==0)
		{
			BoundarySearch(nIndex,V);
		}
		if (V[nIndex]==1)
		{
			V[index]=2;
			V[nIndex]=2;
		}
	}
}

vector<Vector3d> MotionCluster::calClusterBoundaryCenter(Cluster& C,MatrixXd& Aeq,VectorXd& p)
{
	vector<DistPair> distPairs;
	vector<Vector3d> centers;
	for (int i=0;i<C.vIndex.size();i++)
	{
		DistPair dp;
		int index=C.vIndex.at(i);
		Vector3d x=Objs.at(0).vertex[index].Pos;
		dp.index=index;
		dp.dist=(Aeq*x-p).squaredNorm();
		distPairs.push_back(dp);
	}
	sort(distPairs.begin(),distPairs.end(),myCompare);
	for (int t=0;t<frameNum-1;t++)
	{
		Vector3d center(0,0,0);
		int n=0;
		for (int i=0;i<distPairs.size();i++)
		{
			center+=Objs.at(t+1).vertex[distPairs.at(i).index].Pos;
			n++;
			if (distPairs.at(i).dist>10*distPairs.at(0).dist)
				break;
		}	
		centers.push_back(center/n);
		//cout<<n/(double)distPairs.size()<<endl;
	}

	return centers;
}

void MotionCluster::IterativeRigging()
{
	initL();
	cout<<"IR begin!"<<endl;
	cout<<"InitError:"<<calErrorIR()<<endl;
	double lambda=paraL;
	double threshold;
	for (int k=0;k<20;k++)
	{

		threshold=0.01;

		cout<<"cSize:"<<boneNum<<endl;
		for (int i=0;i<2;i++)
		{
			updateWeightIR(false);
			initBoneWeightConstrain(true);
			updateWeightIR(true);
		}
		calJoint();
		cout<<"prevError:"<<calErrorIR()<<endl;
		updateTransformIR(lambda);
		lambda*=1.5;
		//cout<<"afterError:"<<calErrorIR()<<endl;

		bool res1=prune(threshold,true);
		bool res2=updateClusters(true);
		res1=res1||res2;
		
		if (res1)
		{
			while (res1)
			{
				for (int i=0;i<1;i++)
				{
					updateWeight(4,false);
					initBoneWeightConstrain(true);
					updateWeight(4,true);
					//cout<<"prevError:"<<calTotalError()<<endl;
					updateTransform();
					//cout<<"afterError:"<<calTotalError()<<endl;

				}
				res1=updateClusters(true);
			}
			
			TopologyReconstr();
			lambda=1;
		}

		
	}

}

void MotionCluster::initL()
{
	L=new double*[vertexNum];
	for (int i=0;i<vertexNum;i++)
	{
		L[i]=new double[vertexNum];
		memset(L[i],0,vertexNum*sizeof(double));
	}

	for (int i = 0; i < vertexNum; i++)
	{
		double totalD=0;
		L[i][i]=1;
		vector<int> neighbors=Objs.at(0).vertex[i].Neighbor;
		for (int j=0;j<neighbors.size();j++)
		{
			int index=neighbors.at(j);
			L[i][index]=calD(i,index);
			totalD+=L[i][index];
		}
		for (int j=0;j<neighbors.size();j++)
		{
			int index=neighbors.at(j);
			L[i][index]=-L[i][index]/totalD;
		}

	}
}

double MotionCluster::calD(int indexI,int indexK)
{
	double tmpRes=0;
	double res;
	double uIK=(Objs.at(0).vertex[indexI].Pos-Objs.at(0).vertex[indexK].Pos).norm();

	for (int i=1;i<frameNum;i++)
	{
		double vIK=(Objs.at(i).vertex[indexI].Pos-Objs.at(i).vertex[indexK].Pos).norm();
		tmpRes+=(vIK-uIK)*(vIK-uIK);
	}
	res=1/(sqrt(tmpRes/(frameNum-1))+1e-8);
	return res;
}

void MotionCluster::updateWeightIR(bool BCconstructed)
{
	for (int i=0;i<vertexNum;i++)
	{
		//cout<<i<<endl;
		updateVertexWeightIR(i,BCconstructed);
	}
}

void MotionCluster::updateVertexWeightIR(int vIndex,bool BCconstructed)
{
	MatrixXd m_A;
	VectorXd m_b;
	initAandB_IR(m_A,m_b,vIndex);
	//cout<<m_A;
	//cout<<m_b;

	alglib::real_1d_array lb, ub;
	initBound(lb,ub,boneNum);

	alglib::real_2d_array cs;
	alglib::integer_1d_array ct;
	initConstrain(cs,ct,boneNum);
	alglib::real_1d_array sp;
	sp.setlength(boneNum);
	for (int i=0;i<boneNum;i++)
	{
		sp[i]=weight[vIndex][i];
	}
	//cout<<m_A<<endl;
	//cout<<"#####"<<endl;
	//cout<<m_b<<endl;
	VectorXd w=solveLeastSquareIR(m_A,m_b,lb,ub,cs,ct,sp);
	/*for (int i=0;i<boneNum;i++)
	{
		weight[vIndex][i]=w(i);
	}*/
	vector<DistPair> distPairs;
	if (BCconstructed)
	{
		for (int i=0;i<w.size();i++)
		{
			if (BC[vIndex][i]==1){
				DistPair dP;
				dP.dist=calResidue(vIndex,i,w(i));
				dP.index=i;
				distPairs.push_back(dP);
			}	
		}
	}
	else{
		for (int i=0;i<w.size();i++)
		{
			if (w(i)>0){
				DistPair dP;
				dP.dist=calResidue(vIndex,i,w(i));
				dP.index=i;
				distPairs.push_back(dP);
			}	
		}
	}

	sort(distPairs.begin(),distPairs.end(),myCompare);
	vector<int> boneIndex;
	for (int i=0;i<4&&i<distPairs.size();i++)
	{
		boneIndex.push_back(distPairs.at(i).index);
	}

	/*VectorXd refW(boneIndex.size());
	VectorXd bW(boneIndex.size());
	for (int i=0;i<boneIndex.size();i++)
	{
		refW(i)=w(boneIndex.at(i));
		bW(i)=m_b(boneIndex.at(i));
	}
	//cout<<m_b<<endl;
	//cout<<refW<<endl;
	//cout<<bW<<endl;

	//cout<<calLapError(vIndex,boneIndex,refW)<<endl;*/

	initAandB_ex_IR(m_A,m_b,vIndex,boneIndex);
	
	initBound(lb,ub,boneIndex.size());
	initConstrain(cs,ct,boneIndex.size());
	sp.setlength(boneIndex.size());
	for (int i=0;i<boneIndex.size();i++)
	{
		sp[i]=w(boneIndex.at(i));
	}
	w=solveLeastSquareIR(m_A,m_b,lb,ub,cs,ct,sp);
	//cout<<calLapError(vIndex,boneIndex,w)<<endl;

	for (int i=0;i<boneNum;i++)
	{
		weight[vIndex][i]=0;
	}
	for (int i=0;i<boneIndex.size();i++)
	{
		weight[vIndex][boneIndex.at(i)]=w(i);
	}
}

double MotionCluster::calResidue(int vIndex,int bIndex,double w)
{
	double coef=paraW*vertexNum*(frameNum-1);
	double Error=0;
	Vector3d p=Objs.at(0).vertex[vIndex].Pos;
	for (int t=0;t<frameNum-1;t++)
	{
		Vector3d v=Objs.at(t+1).vertex[vIndex].Pos;
		v-=w*transformPoint(p,clusterSet.at(bIndex).transform.at(t));
		Error+=v.squaredNorm();
	}

	//Error=0;
	VectorXd rW(boneNum);
	VectorXd tW(boneNum);
	VectorXd qW(boneNum);
	tW.setZero();
	rW.setZero();
	rW(bIndex)=w;

	vector<int> neighbors=Objs.at(0).vertex[vIndex].Neighbor;
	for (int i=0;i<neighbors.size();i++)
	{
		for (int j=0;j<boneNum;j++)
		{
			qW(j)=weight[neighbors.at(i)][j];
		}
		tW+=qW*L[vIndex][neighbors.at(i)];
	}
	Error+=coef*(rW+tW).squaredNorm();

	/*for (int i=0;i<neighbors.size();i++)
	{
		int nb=neighbors.at(i);
		Error+=-coef*L[vIndex][nb]*(w-weight[nb][bIndex])*(w-weight[nb][bIndex]);
	}*/
	return Error;
}

double MotionCluster::calLapError(int vIndex,vector<int> boneIndex,VectorXd w)
{
	vector<int> neighbors=Objs.at(0).vertex[vIndex].Neighbor;
	for (int i=0;i<neighbors.size();i++)
	{
		int nb=neighbors.at(i);
		VectorXd nW(w.size());
		for (int j=0;j<w.size();j++)
		{
			nW(j)=weight[nb][boneIndex.at(j)];
		}
		w+=nW*L[vIndex][neighbors.at(i)];
	}
	return w.squaredNorm();
}

void MotionCluster::initAandB_IR(MatrixXd& A,VectorXd& b,int vIndex)
{
	vector<int> neighbors=Objs.at(0).vertex[vIndex].Neighbor;
	int neighborNum=neighbors.size();
	double coef=paraW*vertexNum*(frameNum-1);

	MatrixXd Q(3*(frameNum-1),boneNum);
	for (int i=0;i<frameNum-1;i++)
	{
		for (int j=0;j<boneNum;j++)
		{
			Vector3d p=Objs.at(0).vertex[vIndex].Pos;
			p=transformPoint(p,clusterSet.at(j).transform.at(i));
			Q(3*i,j)=p(0);
			Q(3*i+1,j)=p(1);
			Q(3*i+2,j)=p(2);
		}
	}
	A=Q.transpose()*Q;
	//A=MatrixXd::Zero(boneNum,boneNum);

	MatrixXd tA=MatrixXd::Identity(boneNum,boneNum);
	A+=tA*coef;

	/*for (int i=0;i<neighborNum;i++)
	{
		A+=-tA*coef*L[vIndex][neighbors.at(i)];
	}*/
	//cout<<A;
	// calculate b;
	VectorXd l(3*(frameNum-1));
	for (int i=0;i<frameNum-1;i++)
	{
		Vector3d v=Objs.at(i+1).vertex[vIndex].Pos;
		l(3*i)=-v(0);
		l(3*i+1)=-v(1);
		l(3*i+2)=-v(2);
	}
	b=Q.transpose()*l;
	//b=VectorXd::Zero(boneNum);

	VectorXd w(boneNum);
	VectorXd nW(boneNum);

	w.setZero();
	for (int i=0;i<neighborNum;i++)
	{
		for (int j=0;j<boneNum;j++)
		{
			nW(j)=weight[neighbors.at(i)][j];
		}
		w+=nW*L[vIndex][neighbors.at(i)];
	}
	b+=coef*w;
	/*VectorXd w(boneNum);
	for (int i=0;i<boneNum;i++)
	{
		w(i)=weight[vIndex][i];
	}
	b+=-w*coef*L[vIndex][vIndex];

	for (int i=0;i<neighborNum;i++)
	{
		for (int j=0;j<boneNum;j++)
		{
			w(j)=weight[neighbors.at(i)][j];
		}
		b+=w*coef*L[vIndex][neighbors.at(i)];
	}*/
}

void MotionCluster::initAandB_ex_IR(MatrixXd& A,VectorXd& b,int vIndex,vector<int> boneIndex)
{
	int bNum=boneIndex.size();
	vector<int> neighbors=Objs.at(0).vertex[vIndex].Neighbor;
	int neighborNum=neighbors.size();
	double coef=paraW*vertexNum*(frameNum-1);

	MatrixXd Q(3*(frameNum-1),bNum);
	for (int i=0;i<frameNum-1;i++)
	{
		for (int j=0;j<bNum;j++)
		{
			int index=boneIndex.at(j);
			Vector3d p=Objs.at(0).vertex[vIndex].Pos;
			p=transformPoint(p,clusterSet.at(index).transform.at(i));
			Q(3*i,j)=p(0);
			Q(3*i+1,j)=p(1);
			Q(3*i+2,j)=p(2);
		}
	}
	A=Q.transpose()*Q;
	//A=MatrixXd::Zero(bNum,bNum);

	MatrixXd tA=MatrixXd::Identity(bNum,bNum);
	A+=tA*coef;

	/*for (int i=0;i<neighborNum;i++)
	{
		A+=-tA*coef*L[vIndex][neighbors.at(i)];
	}*/

	// calculate b;
	VectorXd l(3*(frameNum-1));
	for (int i=0;i<frameNum-1;i++)
	{
		Vector3d v=Objs.at(i+1).vertex[vIndex].Pos;
		l(3*i)=-v(0);
		l(3*i+1)=-v(1);
		l(3*i+2)=-v(2);
	}
	b=Q.transpose()*l;
	//b=VectorXd::Zero(bNum);

	VectorXd w(bNum);
	VectorXd nW(bNum);

	w.setZero();
	for (int i=0;i<neighborNum;i++)
	{
		for (int j=0;j<bNum;j++)
		{
			nW(j)=weight[neighbors.at(i)][boneIndex.at(j)];
		}
		w+=nW*L[vIndex][neighbors.at(i)];
	}
	b+=w*coef;
	/*VectorXd w(bNum);
	/*for (int i=0;i<bNum;i++)
	{
		w(i)=weight[vIndex][boneIndex.at(i)];
	}
	b+=-w*coef*L[vIndex][vIndex];

	for (int i=0;i<neighborNum;i++)
	{
		for (int j=0;j<bNum;j++)
		{
			w(j)=weight[neighbors.at(i)][boneIndex.at(j)];
		}
		b+=w*coef*L[vIndex][neighbors.at(i)];
	}*/
}

VectorXd MotionCluster::solveLeastSquareIR(MatrixXd& A,VectorXd& b,alglib::real_1d_array& lb,alglib::real_1d_array& ub 
										 ,alglib::real_2d_array& cs,alglib::integer_1d_array& ct,alglib::real_1d_array& sp)
{
	alglib::real_2d_array m_ATA;
	alglib::real_1d_array m_ATb; 
	m_ATA=transformMatrix(A);
	m_ATb=transformVector(b);

	alglib::minqpstate m_qp_optimizer;

	alglib::minqpcreate(A.cols(), m_qp_optimizer);
	//ATA
	alglib::minqpsetquadraticterm(m_qp_optimizer,m_ATA);
	//ATw
	alglib::minqpsetlinearterm(m_qp_optimizer,m_ATb);
	//init qp solver
	alglib::minqpsetalgobleic(m_qp_optimizer,0,0,0,0);

	alglib::minqpsetbc(m_qp_optimizer, lb, ub);
	alglib::minqpsetlc(m_qp_optimizer,cs,ct);
	alglib::minqpsetstartingpoint(m_qp_optimizer,sp);

	alglib::minqpoptimize(m_qp_optimizer);

	alglib::real_1d_array x;
	alglib::minqpreport rep;
	alglib::minqpresults(m_qp_optimizer, x, rep);
	VectorXd w(x.length());
	for (int i=0;i<x.length();i++)
	{
		w(i)=x[i];
	}
	return w;
}

void MotionCluster::updateTransformIR(double lambda)
{
	for (int i=0;i<boneNum;i++)
	{
		updateBoneTransFormIR(i,lambda);
	}
}

void MotionCluster::updateBoneTransFormIR(int bIndex,double lambda)
{
	Cluster& C=clusterSet.at(bIndex);
	//cout<<"Old "<<bIndex<<":"<<calClusterError(C)<<endl;
	vector<EdgeNode> E;
	for (int i=0;i<MSTedgeSet.size();i++)
	{
		EdgeNode& edge=MSTedgeSet.at(i);
		if (edge.v1==bIndex||edge.v2==bIndex)
		{
			E.push_back(edge);
		}
	}

	Vector3d p(0,0,0);
	double w=0;
	for (int i=0;i<vertexNum;i++)
	{
		double tmpW=weight[i][bIndex];
		p+=tmpW*tmpW*Objs.at(0).vertex[i].Pos;
		w+=tmpW*tmpW;
	}
	p/=vertexNum;
	w/=vertexNum;

	for (int i=0;i<E.size();i++)
	{
		BoneJoint& j=joint[E.at(i).v1][E.at(i).v2];
		p+=lambda*j.center;
		w+=lambda;
	}
	p/=w;

	Vector3d* qI=new Vector3d[vertexNum];
	Vector3d* jC=new Vector3d[E.size()];

	for (int t=0;t<C.transform.size();t++)
	{
		MatrixXd trans=C.transform.at(t);
		MatrixXd P(3,vertexNum+E.size());
		MatrixXd Q(3,vertexNum+E.size());

		Vector3d q(0,0,0);

		for (int i=0;i<vertexNum;i++)
		{
			qI[i]=Objs.at(t+1).vertex[i].Pos;
			Vector3d pI=Objs.at(0).vertex[i].Pos;
			for (int j=0;j<boneNum;j++)
			{
				if (j!=bIndex&&weight[i][j]>0)
				{
					qI[i]-=weight[i][j]*transformPoint(pI,clusterSet.at(j).transform.at(t));
				}
			}
			q+=weight[i][bIndex]*qI[i];
		}
		q/=vertexNum;

		for (int i=0;i<E.size();i++)
		{
			EdgeNode& edge=E.at(i);
			BoneJoint& j=joint[edge.v1][edge.v2];
			int index;
			if (edge.v1==bIndex)
				index=edge.v2;
			else
				index=edge.v1;

			jC[i]=transformPoint(j.center,clusterSet.at(index).transform.at(t));
			q+=lambda*jC[i];
		}
		q/=w;
		

		for (int i=0;i<vertexNum;i++)
		{
			Vector3d pI=Objs.at(0).vertex[i].Pos-p;
			P.col(i)=pI*weight[i][bIndex]/vertexNum;
			qI[i]-=weight[i][bIndex]*q;
			Q.col(i)=qI[i]/vertexNum;
			//cout<<pI;
		}
		for (int i=0;i<E.size();i++)
		{
			EdgeNode& edge=E.at(i);
			BoneJoint& j=joint[edge.v1][edge.v2];
			P.col(i+vertexNum)=(j.center-p)*lambda;
			Q.col(i+vertexNum)=(jC[i]-q)*lambda;
		}

		MatrixXd M=P*Q.transpose();
		JacobiSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
		MatrixXd u=svd.matrixU();
		MatrixXd v=svd.matrixV();
		MatrixXd R=v*u.transpose();
		VectorXd T=q-R*p;
		MatrixXd new_trans(3,4);
		new_trans.leftCols(3)=R;
		new_trans.col(3)=T;
		//cout<<trans<<endl;
		//cout<<new_trans<<endl;
		C.transform.at(t)=new_trans;

	}
	//cout<<"New "<<bIndex<<":"<<calClusterError(C)<<endl;
	delete[] qI;
	delete[] jC;
}

double MotionCluster::calErrorIR()
{
	double Error=0;
	double tmpErr=0;

	Error+=calTotalError()/vertexNum/(frameNum-1);
	cout<<"Error1:"<<Error<<endl;
	for (int t=0;t<boneNum;t++)
	{
		for (int i=0;i<vertexNum;i++)
		{
			double tmpR=0;
			for (int j=0;j<vertexNum;j++)
			{
				tmpR+=weight[j][t]*L[i][j];
			}
			tmpErr+=tmpR*tmpR;
		}
	}

	cout<<"Error2:"<<paraW*tmpErr<<endl;
	Error+=paraW*tmpErr;

	tmpErr=0;

	for (int i=0;i<MSTedgeSet.size();i++)
	{
		EdgeNode& edge=MSTedgeSet.at(i);
		BoneJoint& J=joint[edge.v1][edge.v2];
		for (int t=0;t<frameNum-1;t++)
		{
			MatrixXd trans=clusterSet.at(edge.v1).transform.at(t)-clusterSet.at(edge.v2).transform.at(t);
			Vector3d v=transformPoint(J.center,trans);
			tmpErr+=v.squaredNorm();
		}
	}
	cout<<"Error3:"<<tmpErr*paraL<<endl;
	Error+=paraL*tmpErr/(frameNum-1);
	
	return Error;
}

double MotionCluster::calVertexError(int vIndex,VectorXd& w)
{
	double coef=paraW*vertexNum*(frameNum-1);
	double Error=0;
	Vector3d p=Objs.at(0).vertex[vIndex].Pos;
	for (int t=0;t<frameNum-1;t++)
	{
		Vector3d v=Objs.at(t+1).vertex[vIndex].Pos;
		for (int j=0;j<boneNum;j++)
		{
			if (weight[vIndex][j]!=0)
			{
				v-=weight[vIndex][j]*transformPoint(p,clusterSet.at(j).transform.at(t));
			}
		}
		Error+=v.squaredNorm();
	}

	for (int i=0;i<boneNum;i++)
	{
		Error+=coef*(w(i)-weight[vIndex][i])*(w(i)-weight[vIndex][i]);
	}
	

	vector<int> neighbors=Objs.at(0).vertex[vIndex].Neighbor;
	for (int i=0;i<neighbors.size();i++)
	{
		int nb=neighbors.at(i);
		for (int j=0;j<boneNum;j++)
		{
			Error+=coef*L[vIndex][nb]*(weight[vIndex][j]-weight[nb][j])*(weight[vIndex][j]-weight[nb][j]);
		}
		
	}
	return Error;
}

void MotionCluster::initBoneWeightConstrain(bool BBconstrain)
{
	if (BC==NULL)
	{
		BC=new int*[vertexNum];
		for (int i=0;i<vertexNum;i++)
		{
			BC[i]=new int[boneNum];
		}
	}

	for (int i=0;i<vertexNum;i++)
	{
		memset(BC[i],0,boneNum*sizeof(int));
	}

	for (int i=0;i<boneNum;i++)
	{
		calBoneConstrain(i,BBconstrain);
	}

	for (int i=0;i<vertexNum;i++)
	{
		bool flag=false;
		for (int j=0;j<boneNum;j++)
		{
			if (BC[i][j]==1)
			{
				flag=true;
				break;
			}
		}
		if (!flag)
		{
			/*int* V=new int[vertexNum];
			memset(V,0,vertexNum*sizeof(int));
			int count=0;
			getBCfromNeighbor(i,i,V,count);
			delete[] V;
			int c=0;
			for (int j=0;j<boneNum;j++)
			{
				if (BC[i][j]>0)
					c++;
			}
			cout<<i<<":"<<c<<endl;*/
			for (int j=0;j<boneNum;j++)
			{
				BC[i][j]=1;
			}
		}
	}

}

void MotionCluster::getBCfromNeighbor(int vIndex,int curPos,int* V,int& count)
{
	V[curPos]=1;
	vector<int> neighbor=Objs.at(0).vertex[curPos].Neighbor;

	for (int i=0;i<neighbor.size();i++)
	{
		int nb=neighbor.at(i);

		if (V[nb]==1)
			continue;

		for (int t=0;t<boneNum;t++)
		{
			if (BC[nb][t]==1&&BC[vIndex][t]==0)
			{
				BC[vIndex][t]=1;
				count++;
			}
		}
	}

	if (count==0)
	{
		for (int i=0;i<neighbor.size();i++)
		{
			if (V[neighbor.at(i)]==1)
				continue;
			getBCfromNeighbor(vIndex,neighbor.at(i),V,count);
		}
	}
}

vector<int> MotionCluster::findNeighborBones(int bIndex)
{
	vector<int> neighborIndex;
	int* B=new int[boneNum];
	int* V=new int[vertexNum];
	memset(B,0,boneNum*sizeof(int));
	memset(V,-1,vertexNum*sizeof(int));
	vector<int> vIndex;

	for (int i=0;i<clusterSet.size();i++)
	{
		vIndex=clusterSet.at(i).vIndex;
		for (int j=0;j<vIndex.size();j++)
		{
			V[vIndex.at(j)]=i;
		}
	}

	vIndex=clusterSet.at(bIndex).vIndex;
	for (int i=0;i<vIndex.size();i++)
	{
		vector<int> neighbors=(Objs.at(0).vertex[vIndex.at(i)]).Neighbor;
		for (int j=0;j<neighbors.size();j++)
		{
			int bone=V[neighbors.at(j)];
			if (bone>=0)
			{
				B[bone]=1;
			}
			else{
				cout<<"Error!"<<endl;
			}
		}
	}
	B[bIndex]=1;

	for (int i=0;i<boneNum;i++)
	{
		if (B[i]==1)
		{
			neighborIndex.push_back(i);
		}
	}

	delete[] B;
	delete[] V;

	return neighborIndex;
}

void MotionCluster::calBoneConstrain(int bIndex,bool BBconstrain)
{
	vector<int> vIndex=clusterSet.at(bIndex).vIndex;
	int* V=new int[vertexNum];
	int sp=-1;
	memset(V,-1,vertexNum*sizeof(int));

	/*if (BBconstrain)
	{
		Vector3d center(0,0,0);
		BoundingBox bBox;
		bBox.lx=99999;
		bBox.ly=99999;
		bBox.lz=99999;
		bBox.ux=-99999;
		bBox.uy=-99999;
		bBox.uz=-99999;

		for (int i=0;i<vIndex.size();i++)
		{
			Vector3d p=(Objs.at(0).vertex[vIndex.at(i)]).Pos;
			if (p(0)<bBox.lx)
			{
				bBox.lx=p(0);
			}
			if (p(0)>bBox.ux)
			{
				bBox.ux=p(0);
			}
			if (p(1)<bBox.ly)
			{
				bBox.ly=p(1);
			}
			if (p(1)>bBox.uy)
			{
				bBox.uy=p(1);
			}
			if (p(2)<bBox.lz)
			{
				bBox.lz=p(2);
			}
			if (p(2)>bBox.uz)
			{
				bBox.uz=p(2);
			}
		}
		for (int i=0;i<vertexNum;i++)
		{
			Vector3d p=(Objs.at(0).vertex[i]).Pos;

			if (weight[i][bIndex]>0&&pInBoundingBox(p,bBox))
			{
				V[i]=0;
				if (sp==-1)
				{
					sp=i;
				}
			}
			else{
				V[i]=-1;
			}
		}
	}*/
	if (BBconstrain)
	{
		cout<<bIndex<<":";
		vector<int> neighborBones=findNeighborBones(bIndex);
		for (int i=0;i<neighborBones.size();i++)
		{
			cout<<neighborBones.at(i)<<",";
			vector<int> tIndex=clusterSet.at(neighborBones.at(i)).vIndex;
			for (int j=0;j<tIndex.size();j++)
			{
				V[tIndex.at(j)]=1;
			}
		}
		cout<<endl;

		for (int i=0;i<vertexNum;i++)
		{
			Vector3d p=(Objs.at(0).vertex[i]).Pos;

			if (weight[i][bIndex]>0&&V[i]==1)
			{
				V[i]=0;
				if (sp==-1)
				{
					sp=i;
				}
			}
			else{
				V[i]=-1;
			}
		}
	}
	else{
		for (int i=0;i<vertexNum;i++)
		{
			Vector3d p=(Objs.at(0).vertex[i]).Pos;

			if (weight[i][bIndex]>0)
			{
				V[i]=0;
				if (sp==-1)
				{
					sp=i;
				}
			}
		}
	}
	

	

	int maxIndex;
	int maxW=-1;
	int count=0;
	vector<Cluster> tmpSet;
	while(1)
	{
		Cluster A;
		ConnectedSearch(sp,V);
		sp=-1;

		double w=0;
		for (int i=0;i<vertexNum;i++)
		{
			if (V[i]==0&&sp==-1){
				sp=i;
			}
			if (V[i]==1)
			{
				A.vIndex.push_back(i);
				V[i]=-1;
				w+=weight[i][bIndex]*weight[i][bIndex];
			}
		}
		tmpSet.push_back(A);

		if (w>maxW)
		{
			maxW=w;
			maxIndex=count;
		}
		count++;
		if (sp==-1)
			break;
	}	

	/*if (BBconstrain)
	{
		memset(V,-1,vertexNum*sizeof(int));
		vIndex=tmpSet.at(maxIndex).vIndex;
		maxW=-1; 
		for (int j=0;j<vIndex.size();j++)
		{
			int index=vIndex.at(j);
			V[index]=0;

			if(weight[index][bIndex]>maxW){
				maxW=weight[index][bIndex];
				maxIndex=index;
			}
		}
		weightSpread(maxIndex,bIndex,V);
		for (int i = 0; i < vertexNum; i++)
		{
			if (V[i]==1)
			{
				BC[i][bIndex]=1;
			}
		}
	}*/
	
	vIndex=tmpSet.at(maxIndex).vIndex;
	for (int j=0;j<vIndex.size();j++)
	{
		int index=vIndex.at(j);
		BC[index][bIndex]=1;
	}

	
	
}

void MotionCluster::weightSpread(int vIndex,int bIndex,int* V)
{
	vector<int> neighbor=Objs.at(0).vertex[vIndex].Neighbor;
	V[vIndex]=1;
	for (int i=0;i<neighbor.size();i++)
	{
		int nb=neighbor.at(i);
		if (V[nb]!=0)
			continue;
		if (weight[nb][bIndex]-weight[vIndex][bIndex]<0)
		{
			weightSpread(nb,bIndex,V);
		}
	}
}

bool MotionCluster::pInBoundingBox(Vector3d p,BoundingBox& bBox)
{
	double para=1;
	if ((p(0)>(1+para)*bBox.lx-para*bBox.ux)&&(p(0)<(1+para)*bBox.ux-para*bBox.lx)
		&&(p(1)>(1+para)*bBox.ly-para*bBox.uy)&&(p(1)<(1+para)*bBox.uy-para*bBox.ly)
		&&(p(2)>(1+para)*bBox.lz-para*bBox.uz)&&(p(2)<(1+para)*bBox.uz-para*bBox.lz))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void MotionCluster::writeFile(CString& fileName)
{
	char *fName =(LPSTR)(LPCTSTR)fileName;
	FILE* fp=fopen(fName,"w+");

	fprintf(fp,"##############################################################################\n");
	fprintf(fp,"# JOINTS: Num_of_Bones Num_of_Joints\n");
	fprintf(fp,"%d %d\n",boneNum,boneNum-1);
	for (int i = 0; i < MSTedgeSet.size(); i++)
	{
		EdgeNode& edge=MSTedgeSet.at(i);
		BoneJoint& J=joint[edge.v1][edge.v2];
		fprintf(fp,"%d %d    %lf %lf %lf\n",edge.v1,edge.v2,J.center(0),J.center(1),J.center(2));
		fprintf(fp,"%d %d    %lf %lf %lf\n",edge.v2,edge.v1,J.center(0),J.center(1),J.center(2));
	}
	fprintf(fp,"##############################################################################\n");
	fprintf(fp,"# BONES: Num_of_Frames Num_of_Bones\n");
	fprintf(fp,"%d %d\n",frameNum-1,boneNum);
	for (int t=0;t<frameNum-1;t++)
	{
		fprintf(fp,"%d\n",t);
		for (int i=0;i<boneNum;i++)
		{
			fprintf(fp,"%d   ",i);
			MatrixXd& trans=clusterSet.at(i).transform.at(t);
			for (int p=0;p<3;p++)
			{
				for (int q=0;q<4;q++)
				{
					fprintf(fp,"%lf ",trans(p,q));
				}
			}
			fprintf(fp,"\n");
		}
	}

	fprintf(fp,"##############################################################################\n");
	fprintf(fp,"# WEIGHTS: Num_of_Vertices Num_of_Bones\n");
	fprintf(fp,"%d %d\n",vertexNum,boneNum);
	for (int i=0;i<vertexNum;i++)
	{
		fprintf(fp,"%d   ",i);
		for (int j=0;j<boneNum;j++)
		{
			if (weight[i][j]>0)
			{
				fprintf(fp,"%d %lf  ",j,weight[i][j]);
			}
		}
		fprintf(fp,"\n");
	}

	fclose(fp);
}

void MotionCluster::writeTestFile(CString& fileName)
{
	char *fName =(LPSTR)(LPCTSTR)fileName;
	FILE* fp=fopen(fName,"w+");

	fprintf(fp,"##############################################################################\n");
	fprintf(fp,"# JOINTS: Num_of_Bones Num_of_Joints\n");
	fprintf(fp,"%d %d\n",boneNum,boneNum-1);
	for (int i = 0; i < MSTedgeSet.size(); i++)
	{
		EdgeNode& edge=MSTedgeSet.at(i);
		BoneJoint& J=joint[edge.v1][edge.v2];
		fprintf(fp,"%d %d    %lf %lf %lf\n",edge.v1,edge.v2,J.center(0),J.center(1),J.center(2));
		fprintf(fp,"%d %d    %lf %lf %lf\n",edge.v2,edge.v1,J.center(0),J.center(1),J.center(2));
	}
	fprintf(fp,"##############################################################################\n");
	fprintf(fp,"# BONES: Num_of_Frames Num_of_Bones\n");
	fprintf(fp,"%d %d\n",frameNum-1,boneNum);
	for (int t=0;t<frameNum-1;t++)
	{
		fprintf(fp,"%d\n",t);
		for (int i=0;i<boneNum;i++)
		{
			fprintf(fp,"%d   ",i);
			MatrixXd& trans=clusterSet.at(i).transform.at(t);
			for (int p=0;p<3;p++)
			{
				for (int q=0;q<4;q++)
				{
					fprintf(fp,"%lf ",trans(p,q));
				}
			}
			fprintf(fp,"\n");
		}
	}

	fprintf(fp,"##############################################################################\n");
	fprintf(fp,"# WEIGHTS: Num_of_Vertices Num_of_Bones\n");
	fprintf(fp,"%d %d\n",vertexNum,boneNum);

	Vtest=new double[vertexNum];
	for (int i=0;i<vertexNum;i++)
	{
		int bIndex=27;
		int bIndex1=27;
		if (weight[i][bIndex]>0&&weight[i][bIndex1]>0)
		{
			Vtest[i]=weight[i][bIndex];
			fprintf(fp,"%d   ",i);

			for (int j=0;j<boneNum;j++)
			{
				if (weight[i][j]>0)
				{
					fprintf(fp,"%d %lf  ",j,weight[i][j]);
				}
			}

			fprintf(fp,"\n");
		}
		else{
			Vtest[i]=0;
		}
	}
	fclose(fp);
}