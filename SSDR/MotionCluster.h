#pragma once
#include "ObjModel.h"
#include "Cluster.h"
#include "Decompose.h"
#include "alglib/optimization.h"
#include "MST.h"

struct DistPair
{
	int index;
	double dist;
};

using namespace std;
using namespace Eigen;

struct BoundingBox{
	double lx;
	double ux;
	double ly;
	double uy;
	double lz;
	double uz;
};

struct BoneJoint{
	double weight;
	Vector3d center;
};


class MotionCluster
{
public:
	vector<ObjModel> Objs;
	BoneJoint** joint;
	Vector3d Center;
	vector<EdgeNode> MSTedgeSet;
	int boneNum;
	double* Vtest;
private:	
	vector<Cluster> clusterSet;
	double** weight;
	double** L;
	int** BC;
	int frameNum;
	int vertexNum;
	double paraW;
	double paraL;
public:
	MotionCluster(void);
	~MotionCluster(void);
	void loadObjs(CString& strPath);
	void calculateTransformation(Cluster& C);
	void copyHMatrix3D(HMatrix& hmat,Matrix3d& mat);
	void copyFromHMatrix3D(HMatrix& hmat,MatrixXd& mat);
	Vector3d transformPoint(Vector3d& point,MatrixXd& transform);
	void splitCluster(const Cluster& C,vector<Cluster>& cSet);
	bool splitPatch(const Cluster& C,vector<Cluster>& cSet);
	void ConnectedSearch(int index,int* V);
	void BoundarySearch(int index,int* V);
	double calReconstrError(int index,const vector<MatrixXd>& transform);
	double calClusterError(const Cluster& C);
	void generateClusters();
	void SSDRstep();
	void updateWeight(int nzWeightNum,bool BCconstructed);
	void updateTransform();
	void updateVertexWeight(int vIndex,int nzWeightNum,bool BCconstructed);
	void updateBoneTransForm(int bIndex);
	MatrixXd initA(int vIndex);
	VectorXd initb(int vIndex);
	MatrixXd initA_ex(int vIndex,vector<int> boneIndex);
	void initBound(alglib::real_1d_array& lb,alglib::real_1d_array& ub,int size);
	void initConstrain(alglib::real_2d_array& cs,alglib::integer_1d_array& ct,int size);
	alglib::real_2d_array transformMatrix(MatrixXd& A);
	alglib::real_1d_array transformVector(VectorXd& A);
	VectorXd solveLeastSquare(MatrixXd& A,VectorXd& b,alglib::real_1d_array& lb,alglib::real_1d_array& ub
		,alglib::real_2d_array& cs,alglib::integer_1d_array& ct,alglib::real_1d_array& sp);
	double calTotalError();
	void TopologyReconstr();
	void calJoint();
	void calJointCenterAndWeight(int indexA,int indexB);
	vector<Vector3d> calBoundaryCenter(Cluster& A,Cluster& B);
	vector<Vector3d> calClusterBoundaryCenter(Cluster& C,MatrixXd& Aeq,VectorXd& p);
	void pruneBones(vector<int> boneIndex);
	bool prune(double threshold,bool test=false);
	bool updateClusters(bool update);
	void IterativeRigging();
	void initL();
	double calD(int indexI,int indexK);
	void updateWeightIR(bool BCconstructed);
	void updateVertexWeightIR(int vIndex,bool BCconstructed);
	double calResidue(int vIndex,int bIndex,double w);
	void initAandB_IR(MatrixXd& A,VectorXd& b,int vIndex);
	void initAandB_ex_IR(MatrixXd& A,VectorXd& b,int vIndex,vector<int> boneIndex);
	VectorXd solveLeastSquareIR(MatrixXd& A,VectorXd& b,alglib::real_1d_array& lb,alglib::real_1d_array& ub
		,alglib::real_2d_array& cs,alglib::integer_1d_array& ct,alglib::real_1d_array& sp);
	//MatrixXd initA_ex_IR(int vIndex,vector<int> boneIndex);
	//VectorXd initb_ex_IR(int vIndex,vector<int> boneIndex);
	void updateTransformIR(double lambda);
	void updateBoneTransFormIR(int bIndex,double lambda);
	double calErrorIR();
	double calVertexError(int vIndex,VectorXd& w);
	double calEffect(int vIndex,int bIndex,VectorXd w);
	double calResidueEffect(int vIndex,int bIndex,double w);
	void writeFile(CString& fileName);
	void writeTestFile(CString& fileName);
	void initBoneWeightConstrain(bool BBconstrain);
	void calBoneConstrain(int bIndex,bool BBconstrain);
	void getBCfromNeighbor(int vIndex,int curPos,int* V,int& count);
	double calLapError(int vIndex,vector<int> boneIndex,VectorXd w);
	bool pInBoundingBox(Vector3d p,BoundingBox& bBox);
	void weightSpread(int vIndex,int bIndex,int* V);
	vector<int> findNeighborBones(int bIndex);
};

