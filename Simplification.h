#pragma once
#include"util.h"
#include"Pair.h"
#include<vector>
class Simplification {
public:
	void initQ(CMyMesh *mesh);
	vector<Pair> allPair;
	void getAllValidPair(CMyMesh *mesh);
	void getAllCost();
	void contraction(CMyMesh *mesh,Pair p);
	void BuildPairHeap();
	void getCost(Pair &p);
	void simplificate(CMyMesh  *mesh,double ratio);
	
};