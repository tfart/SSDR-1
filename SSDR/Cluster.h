#pragma once
using namespace  Eigen;
using namespace  std;
class Cluster
{
public:
	vector<MatrixXd> transform;
	vector<int> vIndex;
public:
	Cluster(void);
	~Cluster(void);
};

