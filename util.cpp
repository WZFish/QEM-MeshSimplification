#include "util.h"
Vector4d util::getplane(CPoint p1, CPoint p2, CPoint p3) {//计算平面参数
	Vector4d result;
	result(0) = (p2[1] - p1[1])*(p3[2] - p1[2]) - (p3[1] - p1[1])*(p2[2] - p1[2]);
	result(1) = (p2[2] - p1[2])*(p3[0] - p1[0]) - (p2[0] - p1[0])*(p3[2] - p1[2]);
	result(2) = (p2[0] - p1[0])*(p3[1] - p1[1]) - (p3[0] - p1[0])*(p2[1] - p1[1]);
	result(3) = -p1[0] * result(0) - p1[1] * result(1) - p1[2] * result(2);
	double temp = sqrt(result(0)*result(0) + result(1)*result(1) + result(2)*result(2));
	for (int i = 0; i < 4; i++) {
		result(i) = result(i) / temp;
	}
	return result;
}
double util::getdist(CPoint p1, CPoint p2) {
	double dist = sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1])+ (p1[2] - p2[2])*(p1[2] - p2[2]));
	return dist;
}
Matrix4d util::getQ(CVertex *p) {//计算p点的Q
	Matrix4d m;
	m= Matrix4d::Zero();
	for (VertexFaceIterator<CVertex, CEdge, CFace, CHalfEdge> vfiter(p); !vfiter.end(); ++vfiter)
	{
		CFace * pF = *vfiter;
		CPoint p1[3];
		p1[0] = pF->halfedge()->target()->point();
		p1[1]= pF->halfedge()->he_next()->target()->point();
		p1[2]= pF->halfedge()->he_next()->he_next()->target()->point();
		Vector4d result = getplane(p1[0], p1[1], p1[2]);
		m = m + result*result.transpose();
		/*cout << result << endl;
		getchar();*/
	}
	return m;
}
void util::deleteEdge(CMyMesh *mesh,CEdge* e) {//删除边e
	CHalfEdge *pHe = e->halfedge(0);
	mesh->edges().remove((CMyEdge*)e);
	CVertex *v1, *v2;
	v1 = pHe->source();
	v2 = pHe->target();
	list<CMyEdge*> & edgesv1 = (std::list<CMyEdge*> &) v1->edges();
	if (find(edgesv1.begin(), edgesv1.end(), (CMyEdge*)e) != edgesv1.end()) {
		edgesv1.remove((CMyEdge*)e);
	}
	list<CMyEdge*> & edgesv2 = (std::list<CMyEdge*> &) v2->edges();
	if (find(edgesv2.begin(), edgesv2.end(), (CMyEdge*)e) != edgesv2.end()) {
		edgesv2.remove((CMyEdge*)e);
	}
	delete e;
}
CEdge* util::locateEdge(CMyMesh *mesh,CVertex* p1, CVertex*p2) {//找到p1,p2所在的边
	CMyVertex *v = mesh->idVertex(p1->id());
	for (CMyMesh::VertexInHalfedgeIterator vihiter(mesh,v); !vihiter.end(); vihiter++) {
		CHalfEdge *pHe = *vihiter;
		if (pHe->source()->id() == p2->id()) {			
		return pHe->edge();
		}
	}
	return NULL;
}
