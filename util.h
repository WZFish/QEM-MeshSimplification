#pragma once
#include "MyMesh.h"
#include"Point.h"
using namespace MeshLib;


using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
class util {
public:
	static Vector4d getplane(CPoint p1, CPoint p2, CPoint p3);
	static double getdist(CPoint p1, CPoint p2);
	static Matrix4d getQ(CVertex *p);
	static void deleteEdge(CMyMesh *mesh,CEdge* e);
	static CEdge* locateEdge(CMyMesh *mesh,CVertex* p1, CVertex*p2);
};
