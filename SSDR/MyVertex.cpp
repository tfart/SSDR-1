#include "stdafx.h"
#include "MyVertex.h"


MyVertex::MyVertex(void)
{
	cluster=-1;
}


MyVertex::~MyVertex(void)
{
}

void MyVertex::AddNeighbor(int index)
{
	for (int i=0;i<Neighbor.size();i++)
	{
		if (Neighbor.at(i)==index)
			return;
	}

	Neighbor.push_back(index);
}

vector<int> MyVertex::GetNeighbor()
{
	return Neighbor;
}