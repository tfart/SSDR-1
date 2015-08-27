#pragma once
using namespace Eigen;
using namespace std;

class MyVertex
{
public:
	Vector3d Pos;
	vector<int> Neighbor;
	int index;
	int cluster;
public:
	MyVertex(void);
	~MyVertex(void);
	void AddNeighbor(int index);
	vector<int> GetNeighbor();
};

