#include "stdafx.h"
#include "MST.h"


MST::MST(int n)
{
	root=new int[n];
	for (int i = 0; i < n; i++)
	{
		root[i]=i;
	}
}


MST::~MST(void)
{
	delete[] root;
}

int MST::Find(int x)
{
	int i=x;
	while(i!=root[i])
		i=root[i];
	while(i!=root[x])
	{
		x=root[x];
		root[x]=i;
	}
	return i;
}

void MST::Union(int a,int b)
{
	a=Find(a);
	b=Find(b);
	if(a!=b)
		root[a]=b;
}

void MST::Push(EdgeNode& edge)
{
	pq.push(edge);
}

vector<EdgeNode> MST::getEdgeSet()
{
	return edgeSet;
}

void MST::Kruskal()
{
	EdgeNode b;

	while(!pq.empty())
	{
		b=pq.top();
		pq.pop();
		if(Find(b.v1)!=Find(b.v2))
		{
			edgeSet.push_back(b);
			Union(b.v1,b.v2);
		}
	}
}