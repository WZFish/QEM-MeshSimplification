#pragma once
#include"MyMesh.h"
class Pair{
public:
	CVertex *p1;
	CVertex *p2;
	double Cost;
	CPoint v;
	Pair(CVertex *a, CVertex*b) :p1(a), p2(b) {}
};
