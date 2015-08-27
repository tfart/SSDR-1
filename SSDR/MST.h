#pragma once
#include<iostream>
#include<queue>
using namespace std;
struct EdgeNode
{
	int v1;
	int v2;
	double value;
	bool operator<(const EdgeNode &a) const
	{
		return a.value<value;
	}
};

class MST
{
private:
	int *root;
	priority_queue<EdgeNode> pq;
	vector<EdgeNode> edgeSet;
public:
	MST(int n);
	~MST(void);
	int Find(int x);
	void Union(int a,int b);
	void Kruskal();
	void Push(EdgeNode& edge);
	vector<EdgeNode> getEdgeSet();
};

