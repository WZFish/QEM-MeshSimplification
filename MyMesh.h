#ifndef _MY_MESH_
#define _MY_MESH_


#include "Mesh\Vertex.h"
#include "Mesh\Edge.h"
#include "Mesh\Face.h"
#include "Mesh\HalfEdge.h"
#include "Mesh\BaseMesh.h"
#include<Eigen/Dense>
#include "Mesh\boundary.h"
#include "Mesh\iterators.h"
#include "Parser\parser.h"
#include<algorithm>
#include<iostream>
#include<vector>
#include<fstream>
#include<cmath>
#ifndef M_PI
#define M_PI 3.141592653589793238
#endif
using namespace MeshLib;
using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

namespace MeshLib
{
	class CMyVertex;
	class CMyEdge;
	class CMyFace;
	class CMyHalfEdge;
	
	
	class CMyVertex : public CVertex
	{
	public:
		CMyVertex() : m_rgb(1,1,1) {};
		~CMyVertex() {};

		void _from_string() ;
		void _to_string();

		CPoint & rgb() { return m_rgb; };
	protected:
		CPoint m_rgb;
	};

	inline void CMyVertex::_from_string()
	{
		CParser parser(m_string);
		for (std::list<CToken*>::iterator iter = parser.tokens().begin(); iter != parser.tokens().end(); ++iter)
		{
			CToken * token = *iter;
			if (token->m_key == "uv") //CPoint2
			{
				token->m_value >> m_uv;
			}
			if (token->m_key == "rgb") // CPoint
			{
				token->m_value >> m_rgb;
			}
		}
	}

	inline void CMyVertex::_to_string()
	{
		CParser parser(m_string);
		parser._removeToken("uv");

		parser._toString(m_string);
		std::stringstream iss;

		iss << "uv=(" << m_uv[0] << " " << m_uv[1] << ")";

		if (m_string.length() > 0)
		{
			m_string += " ";
		}
		m_string += iss.str();
	}
	
	class CMyEdge : public CEdge
	{
	public:
		CMyEdge() :m_sharp(false) {};
		~CMyEdge() {};

		void _from_string();

		bool & sharp() { return m_sharp; };
	protected:
		bool m_sharp;
	};

	inline void CMyEdge::_from_string()
	{
		CParser parser(m_string);
		for (std::list<CToken*>::iterator iter = parser.tokens().begin(); iter != parser.tokens().end(); ++iter)
		{
			CToken * token = *iter;
			if (token->m_key == "sharp") // bool
			{
				m_sharp = true;
			}
		}
	}

	class CMyFace : public CFace
	{
	public:

		CPoint & normal() { return m_normal; };
	protected:
		CPoint m_normal;
	};

	class CMyHalfEdge : public CHalfEdge
	{
	};

	template<typename V, typename E, typename F, typename H>
	class MyMesh : public CBaseMesh<V, E, F, H>
	{
	public:
		typedef CBoundary<V, E, F, H>					CBoundary;
		typedef CLoop<V, E, F, H>						CLoop;

		typedef MeshVertexIterator<V, E, F, H>			MeshVertexIterator;
		typedef MeshEdgeIterator<V, E, F, H>			MeshEdgeIterator;
		typedef MeshFaceIterator<V, E, F, H>			MeshFaceIterator;
		typedef MeshHalfEdgeIterator<V, E, F, H>		MeshHalfEdgeIterator;

		typedef VertexVertexIterator<V, E, F, H>		VertexVertexIterator;
		typedef VertexEdgeIterator<V, E, F, H>			VertexEdgeIterator;
		typedef VertexFaceIterator<V, E, F, H>			VertexFaceIterator;
		typedef VertexInHalfedgeIterator<V, E, F, H>	VertexInHalfedgeIterator;
		typedef VertexOutHalfedgeIterator<V, E, F, H>	VertexOutHalfedgeIterator;

		typedef FaceVertexIterator<V, E, F, H>			FaceVertexIterator;
		typedef FaceEdgeIterator<V, E, F, H>			FaceEdgeIterator;
		typedef FaceHalfedgeIterator<V, E, F, H>		FaceHalfedgeIterator;

		void output_mesh_info();
		void test_iterator();
		void setStrcture();
		CPoint shengchengPoint();
		bool issame(CPoint p);
		void addPoint(int num);
		double getarea(CVertex *p1, CVertex *p2, CVertex *p3);
		CHalfEdge *locatePoint(CVertex *v);
		void facesplit(CHalfEdge *face, CVertex *v);
		double getR(CPoint p1, CPoint p2, CPoint p3);
		double getdist(CPoint p1, CPoint p2);
		CPoint getCenter(CPoint p1, CPoint p2, CPoint p3);
		bool isin(CPoint p1, CPoint p2, CPoint p3, CPoint v);
		bool LegalizeEdge(CVertex *v, CHalfEdge *e);
		void edgeswap(CVertex *v, CHalfEdge *e);
		bool issameface(vector<int> face);
		void getface();
		void save();
		void Sort(vector<int>& data);
		void getbounddisk();
		void getweight();
		double calE();
		void init1();
		double getangle(CPoint p1, CPoint p2);
		void Harmonic_map_on_disk(double deltaE);
		void getbound_quad();
		void Harmonic_map_on_quad(double deltaE);
		//void 
	public:
		vector<double> boundweight;
		
		vector<CVertex*> bounddisk;
		vector<CVertex*> boundquad;
		
		CVertex* initVertex[4];
		vector<CPoint> existPoint;
		vector<vector<int>> listOfFace;
		vector<vector<int>> existFace;
		CHalfEdge *initHalfEdge[6];
		CPoint initPoint[4];
		vector<CVertex*> listOfVertex;
		vector<CHalfEdge*> listOfHalfEdge;	
	};
	template<typename V, typename E, typename F, typename H>
	double MyMesh<V, E, F, H>::getangle(CPoint p1, CPoint p2) {
		double a = p1*p2;
		double b = p1.norm();
		double c = p2.norm();
		double cosine = a / (b*c);
		double angle = acos(cosine);
		return angle;
	}
	bool cmp(CVertex *p1, CVertex *p2) {
		if (p1->point()[1] > 0 && p2->point()[1] > 0) {
			return p1->point()[0] > p2->point()[0];
		}
		else if (p1->point()[1] > 0 && p2->point()[1] < 0) {
			return p1->point()[1] > p2->point()[1];
		}
		else if (p1->point()[1] < 0 && p2->point()[1] < 0) {
			return p1->point()[0] < p2->point()[0];
		}
		else {
			return p1->point()[1] > p2->point()[1];
		}
	}
	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::getbounddisk() {

		
		for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
			CVertex *pv = *viter;
			if (pv->boundary()) {
				bounddisk.push_back(pv);
			}
			
			
		}
		sort(bounddisk.begin(), bounddisk.end(), cmp);
		/*for (int i = 0; i < bounddisk.size(); i++) {
			//if(bounddisk[i])
			cout << bounddisk[i]->point()[0] << " " << bounddisk[i]->point()[1] << endl;
		}*/
		double dist=0;
		vector<double>distOfEveryEdge;
		for (int i = 1; i < bounddisk.size(); i++) {
			//if(bounddisk[i])
			//cout << bounddisk[i]->point()[0] << " " << bounddisk[i]->point()[1] << endl;
			double a= getdist(bounddisk[i]->point(), bounddisk[i - 1]->point());
			distOfEveryEdge.push_back(a);
			dist = dist + getdist(bounddisk[i]->point(), bounddisk[i - 1]->point());

		}
		dist = dist + getdist(bounddisk[bounddisk.size()-1]->point(), bounddisk[0]->point());
		distOfEveryEdge.push_back(getdist(bounddisk[bounddisk.size()-1]->point(), bounddisk[0]->point()));
		for (int i = 0; i < distOfEveryEdge.size(); i++) {
			double s = 0;
			for (int j = 0; j < i; j++) {
				s = s + distOfEveryEdge[j];
			}
			double theta =2*M_PI*s / dist;
			CPoint p;
			p[0] = cos(theta);
			p[1] = sin(theta);
			p[2] = 0;
			bounddisk[i]->point() = p;
		}

		cout << "boundpoint:"<<bounddisk.size()<<"edgenum"<<distOfEveryEdge.size()<<endl;
		/*for (int i = 0; i < bounddisk.size(); i++) {
			//if(bounddisk[i])
			cout << bounddisk[i]->point()[0] << " " << bounddisk[i]->point()[1] << endl;
		}*/
		//double dist = 0;
		//vector<double>distOfEveryEdge;
		//for (MeshEdgeIterator eiter(this); !eiter.end(); ++eiter)
		//{

		//	CEdge* pE = *eiter;
		//	if (pE->boundary()) {
		//		CHalfEdge *pHe = pE->halfedge(0);
		//		CVertex *v1 = pHe->he_next()->he_next()->target();
		//		CVertex *v2 = pHe->target();
		//		double dist1= getdist(v1->point(), v2->point());
		//		distOfEveryEdge.push_back(dist1);
		//		bounddisk.push_back(v1);
		//		dist += dist1;
		//	}
		//}
		//cout << distOfEveryEdge.size() << endl;
		//for (int i = 0; i < distOfEveryEdge.size(); i++) {
		//	double s = 0;
		//	for (int j = 0; j < i; j++) {
		//		s = s + distOfEveryEdge[j];
		//	}
		//	double theta =2*M_PI*s / dist;
		//	CPoint p;
		//	//cout << cos(0.000)<<endl;
		//	/*double a= cos(theta);
		//	double b = sin(theta);
		//	*/
		//	p[0] = cos(theta);
		//	p[1] = sin(theta);
		//	p[2] = 0;
		//	//cout << p[0] << " " << p[1] << endl;
		//	bounddisk[i]->point() = p;
		//}


	}
	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::getbound_quad() {
		for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
			CVertex *pv = *viter;
			if (pv->boundary()) {
				boundquad.push_back(pv);
			}
		}
		cout << boundquad.size() << endl;
		sort(boundquad.begin(), boundquad.end(), cmp);
		int index[4];
		index[0] = 0;
		for (int i = 1; i < 4; i++) {
			index[i] = boundquad.size() / 4 * i;
			cout << index[i] << endl;
		}
		for (int i = 0; i < 4; i++) {
			cout << boundquad[index[i]]->point()[0] << " " << boundquad[index[i]]->point()[1] << endl;
		}
		boundquad[index[0]]->point()[0] = 1;
		boundquad[index[0]]->point()[1] = 1;
		boundquad[index[1]]->point()[0] = 0;
		boundquad[index[1]]->point()[1] = 1;
		boundquad[index[2]]->point()[0] = 0;
		boundquad[index[2]]->point()[1] = 0;
		boundquad[index[3]]->point()[0] = 1;
		boundquad[index[3]]->point()[1] = 0;
		double dist = 0;
		vector<double> distOfEveryEdge;
		for (int i = index[0] + 1; i <= index[1]; i++) {
			double a= getdist(boundquad[i]->point(), boundquad[i - 1]->point());
			dist = dist +a;
			distOfEveryEdge.push_back(a);
			//cout << a << endl;
		}
		//cout << dist<<endl;
		for (int i = index[0]; i < index[1]; i++) {
			double s = 0;
			for (int j = index[0]; j < i; j++) {
				s = s + distOfEveryEdge[j];
			}
			boundquad[i]->point()[0] = 1 - s / dist;
			boundquad[i]->point()[1] = 1;

		}
		dist = 0;
		//distOfEveryEdge.clear();
		for (int i = index[1] + 1; i <= index[2]; i++) {
			double a = getdist(boundquad[i]->point(), boundquad[i - 1]->point());
			dist = dist + a;
			distOfEveryEdge.push_back(a);
			//cout << a << endl;
		}
		//cout << dist << endl;
		for (int i = index[1]; i < index[2]; i++) {
			double s = 0;
			for (int j = index[1]; j < i; j++) {
				s = s + distOfEveryEdge[j];
			}
			boundquad[i]->point()[0] = 0;
			boundquad[i]->point()[1] = 1 - s / dist;

		}
		dist = 0;
		//distOfEveryEdge.clear();
		for (int i = index[2] + 1; i <= index[3]; i++) {
			double a = getdist(boundquad[i]->point(), boundquad[i - 1]->point());
			dist = dist + a;
			distOfEveryEdge.push_back(a);
			//cout << a << endl;
		}
		//cout << dist << endl;
		for (int i = index[2]; i < index[3]; i++) {
			double s = 0;
			for (int j = index[2]; j < i; j++) {
				s = s + distOfEveryEdge[j];
			}
			boundquad[i]->point()[0] = s / dist;
			boundquad[i]->point()[1] =0 ;

		}
		dist = 0;
		//distOfEveryEdge.clear();
		for (int i = index[3] + 1; i < boundquad.size(); i++) {
			double a = getdist(boundquad[i]->point(), boundquad[i - 1]->point());
			dist = dist + a;
			distOfEveryEdge.push_back(a);
		}
		double a= getdist(boundquad[boundquad.size() - 1]->point(), boundquad[0]->point());
		dist = dist + a;
		distOfEveryEdge.push_back(a);
		for (int i = index[3]; i < boundquad.size(); i++) {
			double s = 0;
			for (int j = index[3] ; j < i; j++) {
				s = s + distOfEveryEdge[j];
			}
			boundquad[i]->point()[0] = 1;


			boundquad[i]->point()[1] = s / dist;

		}
		boundquad[index[0]]->point()[0] = 1;
		boundquad[index[0]]->point()[1] = 1;
		boundquad[index[1]]->point()[0] = 0;
		boundquad[index[1]]->point()[1] = 1;
		boundquad[index[2]]->point()[0] = 0;
		boundquad[index[2]]->point()[1] = 0;
		boundquad[index[3]]->point()[0] = 1;
		boundquad[index[3]]->point()[1] = 0;
		for (int i = 0; i < boundquad.size(); i++) {
			cout << boundquad[i]->point()[0] << " " << boundquad[i]->point()[1] << endl;
		}
		
	}
	
	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::getweight() {
		for (MeshEdgeIterator eiter(this); !eiter.end(); ++eiter)
		{
			
			CEdge* pE = *eiter;
			if (pE->boundary()) {
				CPoint p1 = pE->halfedge(0)->he_next()->target()->point();
				CPoint p2 = pE->halfedge(0)->he_prev()->target()->point();
				CPoint p3 = pE->halfedge(0)->target()->point();
				double angle = getangle(p1 - p2, p1 - p3);
				//double k = tan(M_PI / 2 - angle);
				double k = 1 / tan(angle);
				pE->setk(k);
			}
			else {
				CPoint p1 = pE->halfedge(0)->he_next()->target()->point();
				CPoint p2 = pE->halfedge(0)->he_prev()->target()->point();
				CPoint p3 = pE->halfedge(0)->target()->point();
				double angle1 = getangle(p1 - p2, p1 - p3);
				CPoint p11 = pE->halfedge(1)->he_next()->target()->point();
				CPoint p22 = pE->halfedge(1)->he_prev()->target()->point();
				CPoint p33 = pE->halfedge(1)->target()->point();
				double angle2 = getangle(p11 - p22, p11 - p33);
				//double k = tan(M_PI / 2 - angle1)+ tan(M_PI / 2 - angle2);
				double k = 1 / tan(angle1) + 1 / tan(angle2);
				pE->setk(k);
			}
			// you can do something to the edge here
			// ...
		}
	}

	template<typename V, typename E, typename F, typename H>
	double MyMesh<V, E, F, H>::calE() {
		double E = 0;
		for (MeshEdgeIterator eiter(this); !eiter.end(); ++eiter) {
			
			CEdge *pE = *eiter;
			
			CVertex *p1 = pE->halfedge(0)->he_next()->he_next()->target();
			CVertex *p2 = pE->halfedge(0)->target();
			E = E + pE->getk()*(((p1->point()[0] - p2->point()[0])*(p1->point()[0] - p2->point()[0])) + ((p1->point()[1] - p2->point()[1])*(p1->point()[1] - p2->point()[1]))+ ((p1->point()[2] - p2->point()[2])*(p1->point()[2] - p2->point()[2])));
			

		}
		return E;
		
	}
	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::init1() {
		for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
			CVertex *pv = *viter;
			if (!pv->boundary()) {
				pv->point()[0] = 0;
				pv->point()[1] = 0;
				pv->point()[2] = 0;
			}
			
		}
	}
	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::Harmonic_map_on_disk(double deltaE) {
		getbounddisk();
		getweight();
		//init1();
		for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
			CVertex *pv = *viter;
			if (!pv->boundary()) {
				pv->point()[0] = 0;
				pv->point()[1] = 0;
				pv->point()[2] = 0;
			}
			/*else {
				pv->point()[2] = 0;
			}*/
		}
		double newE=0, oldE=0;
		newE = calE();
		do {
			oldE = newE;
			//cout <<"oldE:"<< oldE << endl;
			for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
				CMyVertex *pv = *viter;
				double sumk = 0;
				CPoint P;
				P[0] = P[1] = P[2] = 0;
				/*if (!pv->boundary()) {
					for (VertexEdgeIterator veiter(pv); !veiter.end(); ++veiter) {
						
						
						CEdge *pE = *veiter;
						CHalfEdge *pHe = pE->halfedge(0);
						CVertex *v1;
						if (pHe->target() != pv) {
							v1 = pHe->target();
						}
						else {
							pHe = pHe->he_sym();
							v1 = pHe->target();
						}
						sumk = sumk + pE->getk();
						P[0] =P[0]+ v1->point()[0] * pE->getk();
						P[1] = P[1] + v1->point()[1] * pE->getk();
					}
					P[0] = P[0] / sumk;
					P[1] = P[1] / sumk;
					pv->point() = P;
					
				}*/
				if (!pv->boundary()) {
					for (VertexVertexIterator vviter(pv); !vviter.end(); ++vviter) {
						CMyVertex *v1 = *vviter;
						CEdge *e1 = this->vertexEdge(pv, v1);
						sumk = sumk + e1->getk();
						P[0] = P[0] + v1->point()[0] * e1->getk();
						P[1] = P[1] + v1->point()[1] * e1->getk();
					}
					P[0] = P[0] / sumk;
					P[1] = P[1] / sumk;
					pv->point() = P;

				}
			}
			newE = calE();
			cout << "newE:" << newE << endl;

		} while (abs(oldE - newE) > deltaE);
		

	}
	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::Harmonic_map_on_quad(double deltaE) {
		getbound_quad();
		getweight();
		//init1();
		for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
			CVertex *pv = *viter;
			if (!pv->boundary()) {
				pv->point()[0] = 0.5;
				pv->point()[1] = 0.5;
				pv->point()[2] = 0;
			}
			else {
				pv->point()[2] = 0;
			}
		}
		double newE = 0, oldE = 0;
		newE = calE();
		do {
			oldE = newE;
			//cout <<"oldE:"<< oldE << endl;
			for (MeshVertexIterator viter(this); !viter.end(); ++viter) {
				CMyVertex *pv = *viter;
				double sumk = 0;
				CPoint P;
				P[0] = P[1] = P[2] = 0;
				if (!pv->boundary()) {
					for (VertexEdgeIterator veiter(pv); !veiter.end(); ++veiter) {


						CEdge *pE = *veiter;
						CHalfEdge *pHe = pE->halfedge(0);
						CVertex *v1;
						if (pHe->target() != pv) {
							v1 = pHe->target();
						}
						else {
							pHe = pHe->he_sym();
							v1 = pHe->target();
						}
						sumk = sumk + pE->getk();
						P[0] = P[0] + v1->point()[0] * pE->getk();
						P[1] = P[1] + v1->point()[1] * pE->getk();
					}
					P[0] = P[0] / sumk;
					P[1] = P[1] / sumk;
					pv->point() = P;

				}
			}
			newE = calE();
			cout << "newE:" << newE << endl;

		} while (abs(oldE - newE) > deltaE);

	}











	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::Sort(vector<int>& data) {
		int min = 0;
		for (int i = 0; i < data.size(); i++) {
			min = data[i] < data[min] ? i : min;
		}
		if (min == 1) {
			swap(data[0], data[1]);
			swap(data[1], data[2]);
		}
		else if (min == 2) {
			swap(data[0], data[1]);
			swap(data[0], data[2]);
		}
	}

	typedef MyMesh<CMyVertex, CMyEdge, CMyFace, CMyHalfEdge> CMyMesh;
	template<typename V, typename E, typename F, typename H>
	void  MeshLib::MyMesh<V, E, F, H>::setStrcture() {
		initPoint[0][0] = 0;
		initPoint[0][1] = 0;

		initPoint[1][0] = 200;
		initPoint[1][1] = 100;

		initPoint[2][0] = 0;
		initPoint[2][1] = 100;

		initPoint[3][0] =200;
		initPoint[3][1] = 0;

		for (int i = 0; i < 4; i++) {
			initVertex[i] = createVertex(i + 1);
			initVertex[i]->point() = initPoint[i];
			initVertex[i]->id() = i + 1;
		}
		for (int i = 0; i < 4; i++) {
			initHalfEdge[i] = new CHalfEdge;
			initHalfEdge[i]->vertex() = initVertex[i];
			initVertex[i]->halfedge() = initHalfEdge[i];
		}
		initHalfEdge[4] = new CHalfEdge;
		initHalfEdge[4]->vertex() = initVertex[1];
		initVertex[1]->halfedge() = initHalfEdge[4];

		initHalfEdge[5] = new CHalfEdge;
		initHalfEdge[5]->vertex() = initVertex[0];
		initVertex[0]->halfedge() = initHalfEdge[5];

		initHalfEdge[0]->he_prev() = initHalfEdge[2];
		initHalfEdge[0]->he_next() = initHalfEdge[1];

		initHalfEdge[1]->he_prev() = initHalfEdge[0];
		initHalfEdge[1]->he_next() = initHalfEdge[2];

		initHalfEdge[2]->he_prev() = initHalfEdge[1];
		initHalfEdge[2]->he_next() = initHalfEdge[0];

		initHalfEdge[3]->he_prev() = initHalfEdge[5];
		initHalfEdge[3]->he_next() = initHalfEdge[4];

		initHalfEdge[4]->he_prev() = initHalfEdge[3];
		initHalfEdge[4]->he_next() = initHalfEdge[5];

		initHalfEdge[5]->he_prev() = initHalfEdge[4];
		initHalfEdge[5]->he_next() = initHalfEdge[3];

		CEdge* edge[5];
		edge[0] = new CEdge;
		edge[0]->halfedge(0) = initHalfEdge[0];
		initHalfEdge[0]->edge() = edge[0];

		edge[2] = new CEdge;
		edge[2]->halfedge(0) = initHalfEdge[2];
		initHalfEdge[2]->edge() = edge[2];

		edge[3] = new CEdge;
		edge[3]->halfedge(0) = initHalfEdge[3];
		initHalfEdge[3]->edge() = edge[3];

		edge[4] = new CEdge;
		edge[4]->halfedge(0) = initHalfEdge[4];
		initHalfEdge[4]->edge() = edge[4];

		edge[5] = new CEdge;
		edge[5]->halfedge(0) = initHalfEdge[1];
		edge[5]->halfedge(1) = initHalfEdge[5];
		initHalfEdge[1]->edge() = edge[5];
		initHalfEdge[5]->edge() = edge[5];
		for (int i = 0; i < 4; i++) {
			listOfVertex.push_back(initVertex[i]);
		}
	}
	template<typename V, typename E, typename F, typename H>
	CPoint  MeshLib::MyMesh<V, E, F, H>::shengchengPoint() {
		CPoint p;
		p[0] = rand() % 199 + 1;
		p[1] = rand() % 99 + 1;
		return p;
	}
	template<typename V, typename E, typename F, typename H>
	bool  MeshLib::MyMesh<V, E, F, H>::issame(CPoint p) {
		for (int i = 0; i < existPoint.size(); i++) {
			if (p[0] == existPoint[i][0] && p[1] == existPoint[i][1]) {
				return true;
			}
		}
		return false;
	}
	template<typename V, typename E, typename F, typename H>
	void  MeshLib::MyMesh<V, E, F, H>::addPoint(int num) {
		for (int i = 0; i < num; i++) {
			CPoint p = shengchengPoint();
			CHalfEdge *pface;
			if (!issame(p)) {
				CVertex *temp = createVertex(i + 5);
				temp->point() = p;
				temp->id() = i + 5;
				existPoint.push_back(p);
				listOfVertex.push_back(temp);
				pface = locatePoint(temp);
				CHalfEdge *temp0, *temp1;
				temp0 = pface->he_next();
				temp1 = temp0->he_next();
				facesplit(pface, temp);
				LegalizeEdge(temp, pface);
				LegalizeEdge(temp, temp0);
				LegalizeEdge(temp, temp1);
			}
			else {
				i--;
			}

		}
	}
	template<typename V, typename E, typename F, typename H>
	double  MeshLib::MyMesh<V, E, F, H>::getarea(CVertex *p1, CVertex *p2, CVertex *p3) {
		return 0.5*(p1->point()[0] * p2->point()[1] + p1->point()[1]*p3->point()[0] + p2->point()[0] * p3->point()[1] - p2->point()[1] * p3->point()[0] - p1->point()[1] * p2->point()[0] - p3->point()[1]*p1->point()[0]);
	}
	template<typename V, typename E, typename F, typename H>
	CHalfEdge * MeshLib::MyMesh<V, E, F, H>::locatePoint(CVertex *v) {
		CVertex *Point[3];//面的三个点
		CHalfEdge *halfedge[3];//三个半边构成面
							   //选择初始面
		halfedge[0] = initVertex[0]->halfedge();
		halfedge[1] = halfedge[0]->he_next();
		halfedge[2] = halfedge[1]->he_next();
		for (int i = 0; i < 3; i++) {
			Point[i] = halfedge[i]->target();
			//cout << Point[i]->point()[0]<<" "<< Point[i]->point()[1] << endl;
		}
		
		
			
			bool flag;
			do {
				double alpha[3];
				double S = getarea(Point[0], Point[1], Point[2]);
				//cout << S << endl;
				for (int i = 0; i < 3; i++) {
					alpha[i] = getarea(v,Point[(1 + i) % 3], Point[(2 + i) % 3]) / S;
					//cout << alpha[i] << endl;
				}
				flag = true;
				for (int j = 0; j < 3; j++) {
					if (alpha[j] < 0) {
						while (halfedge[(j + 2) % 3]->he_sym() == NULL) {
							halfedge[(j + 2) % 3]=halfedge[(j + 2) % 3]->he_next();
						}
						
						halfedge[j] = halfedge[(j + 2) % 3]->he_sym()->he_next();
						Point[j] = halfedge[j]->target();
						//cout << Point[j]->point()[0] << " " << Point[j]->point()[1] << endl;
						halfedge[(j + 1) % 3] = halfedge[j]->he_next();
						Point[(j + 1) % 3] = halfedge[(j + 1) % 3]->target();
						//cout << Point[(j + 1) % 3]->point()[0] << " " << Point[(j + 1) % 3]->point()[1] << endl;
						halfedge[(j + 2) % 3] = halfedge[(j + 1) % 3]->he_next();
						Point[(j + 2) % 3] = halfedge[(j + 2) % 3]->target();
						//cout << Point[(j + 2) % 3]->point()[0] << " " << Point[(j + 2) % 3]->point()[1] << endl;
						flag = false;
						break;
					}
				}
			
			
			} while (!flag);
			return halfedge[0];
			
		
	}
	template<typename V, typename E, typename F, typename H>
	void  MeshLib::MyMesh<V, E, F, H>::facesplit(CHalfEdge *face, CVertex *v) {
		CHalfEdge *HalfEdge[3];
		CVertex *Point[3];
		CHalfEdge *newEdge[6];

		HalfEdge[0] = face;
		Point[0] = HalfEdge[0]->target();
		HalfEdge[1] = HalfEdge[0]->he_next();
		Point[1] = HalfEdge[1]->target();
		HalfEdge[2] = HalfEdge[1]->he_next();
		Point[2] = HalfEdge[2]->target();
		for (int i = 0; i < 3; i++) {
			newEdge[i] = new CHalfEdge;
			newEdge[i]->vertex() = Point[i];
			Point[i]->halfedge() = newEdge[i];
		}
		for (int i = 3; i < 6; i++) {
			newEdge[i] = new CHalfEdge;
			newEdge[i]->vertex() = v;
			v->halfedge() = newEdge[i];
		}
		HalfEdge[0]->he_prev() = newEdge[2];
		HalfEdge[0]->he_next() = newEdge[3];

		HalfEdge[1]->he_prev() = newEdge[0];
		HalfEdge[1]->he_next() = newEdge[4];

		HalfEdge[2]->he_prev() = newEdge[1];
		HalfEdge[2]->he_next() = newEdge[5];

		newEdge[0]->he_prev() = newEdge[4];
		newEdge[0]->he_next() = HalfEdge[1];

		newEdge[1]->he_prev() = newEdge[5];
		newEdge[1]->he_next() = HalfEdge[2];

		newEdge[2]->he_prev() = newEdge[3];
		newEdge[2]->he_next() = HalfEdge[0];


		newEdge[3]->he_prev() = HalfEdge[0];
		newEdge[3]->he_next() = newEdge[2];

		newEdge[4]->he_prev() = HalfEdge[1];
		newEdge[4]->he_next() = newEdge[0];

		newEdge[5]->he_prev() = HalfEdge[2];
		newEdge[5]->he_next() = newEdge[1];

		CEdge *Edge[3];
		for (int i = 0; i < 3; i++) {
			Edge[i] = new CEdge;
			Edge[i]->halfedge(0) = newEdge[i];
			Edge[i]->halfedge(1) = newEdge[i+3];
			newEdge[i]->edge() = Edge[i];
			newEdge[i+3]->edge() = Edge[i];
		}
		/*Edge[0] = new CEdge;
		Edge[0]->halfedge(0) = newEdge[0];
		Edge[0]->halfedge(1) = newEdge[3];
		newEdge[0]->edge() = Edge[0];
		newEdge[3]->edge() = Edge[0];


		Edge[1] = new CEdge;
		Edge[1]->halfedge(0) = newEdge[4];
		Edge[1]->halfedge(1) = newEdge[1];
		newEdge[4]->edge() = Edge[1];
		newEdge[1]->edge() = Edge[1];

		Edge[2] = new CEdge;
		Edge[2]->halfedge(0) = newEdge[2];
		Edge[2]->halfedge(1) = newEdge[5];
		newEdge[2]->edge() = Edge[2];
		newEdge[5]->edge() = Edge[2];
		for (int i = 0; i < 6; i++) {
			newEdge[i]->he_sym() = newEdge[(i + 3) % 6];
		}*/

		


	}
	template<typename V, typename E, typename F, typename H>
	double MeshLib::MyMesh<V, E, F, H>::getR(CPoint p1, CPoint p2, CPoint p3) {
		double dist12 = getdist(p1, p2);
		double dist13 = getdist(p1, p3);
		double dist23 = getdist(p2, p3);
		double p = (dist12 + dist13 + dist23) / 2.0;
		double area = sqrt(p*(p - dist12)*(p - dist13)*(p - dist23));
		double r = dist12*dist23*dist13 / (4 * area);
		return r;
	}
	template<typename V, typename E, typename F, typename H>
	double  MeshLib::MyMesh<V, E, F, H>::getdist(CPoint p1, CPoint p2) {
		double dist = sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]));
		return dist;
	}
	template<typename V, typename E, typename F, typename H>
	CPoint MeshLib::MyMesh<V, E, F, H>::getCenter(CPoint p1, CPoint p2, CPoint p3) {
		double x1, x2, x3, y1, y2, y3;
		x1 = p1[0];
		y1 = p1[1];
		x2 = p2[0];
		y2 = p2[1];
		x3 = p3[0];
		y3 = p3[1];
		double t1 = x1*x1 + y1*y1;
		double t2 = x2*x2 + y2*y2;
		double t3 = x3*x3 + y3*y3;
		double temp = x1*y2 + x2*y3 + x3*y1 - x1*y3 - x2*y1 - x3*y2;
		double x = (t2*y3 + t1*y2 + t3*y1 - t2*y1 - t3*y2 - t1*y3) / temp / 2;
		double y = (t3*x2 + t2*x1 + t1*x3 - t1*x2 - t2*x3 - t3*x1) / temp / 2;
		CPoint c;
		c[0] = x;
		c[1] = y;
		return c;
	}
	template<typename V, typename E, typename F, typename H>
	bool  MeshLib::MyMesh<V, E, F, H>::isin(CPoint v,CPoint p1, CPoint p2, CPoint p3) {
		double r = getR(v,p1, p2);
		//cout << r << endl;
		CPoint c = getCenter(v,p1, p2);
		//cout << c[0] << " " << c[1] << endl;
		double dist = getdist(c, p3);
		if (dist <= r) {
			return true;
		}
		else {
			return false;
		}
	}
	template<typename V, typename E, typename F, typename H>
	bool  MeshLib::MyMesh<V, E, F, H>::LegalizeEdge(CVertex *v, CHalfEdge *e) {
		if (e->he_sym() == NULL) {
			return false;
		}
		CVertex *Point[3];
		Point[0] = e->he_sym()->target();
		Point[1] = e->target();
		Point[2] = e->he_sym()->he_next()->target();
		CHalfEdge *temp0 = e->he_next();
		CHalfEdge *temp1 = temp0->he_next();
		if (isin(v->point(),Point[0]->point(), Point[1]->point(), Point[2]->point() )) {
			edgeswap(v, e);
			cout << "swap" << endl;
			LegalizeEdge(v, temp0);
			
			LegalizeEdge(v, temp1);
			return true;
		}
		else {
			return false;
		}
	}
	template<typename V, typename E, typename F, typename H>
	void  MeshLib::MyMesh<V, E, F, H>::edgeswap(CVertex *v, CHalfEdge *e) {
		if (NULL == e->he_sym()) {
			return;
		}
		CVertex *Point[3];
		Point[0] = e->he_sym()->target();
		Point[1] = e->target();
		Point[2] = e->he_sym()->he_next()->target();
		CHalfEdge *origenhe[4];
		origenhe[0] = e->he_next();
		origenhe[1] = origenhe[0]->he_next();
		origenhe[2] = e->he_sym()->he_next();
		origenhe[3] = origenhe[2]->he_next();
		CHalfEdge *newhe[2];
		newhe[0] = new CHalfEdge;
		newhe[0]->vertex() = v;
		v->halfedge() = newhe[0];
		newhe[1] = new CHalfEdge;
		newhe[1]->vertex() = Point[2];
		Point[2]->halfedge() = newhe[1];


		origenhe[0]->he_next() = newhe[1];
		origenhe[0]->he_prev() = origenhe[3];
		origenhe[1]->he_prev() = newhe[0];
		origenhe[1]->he_next() = origenhe[2];
		origenhe[2]->he_prev() = origenhe[1];
		origenhe[2]->he_next() = newhe[0];
		origenhe[3]->he_prev() = newhe[1];
		origenhe[3]->he_next() = origenhe[0];
		newhe[0]->he_prev() = origenhe[2];
		newhe[0]->he_next() = origenhe[1];

		newhe[1]->he_prev() = origenhe[0];
		newhe[1]->he_next() = origenhe[3];
		CEdge *edge = new CEdge;
			
		edge->halfedge(0) = newhe[0];
		edge->halfedge(1) = newhe[1];
		
		newhe[0]->edge() = edge;
		newhe[1]->edge() = edge;
		
	}
	template<typename V, typename E, typename F, typename H>
	bool  MeshLib::MyMesh<V, E, F, H>::issameface(vector<int> face) {
		for (int i = 0; i < existFace.size(); i++) {
			if (existFace[i][0] == face[0] && existFace[i][1] == face[1] && existFace[i][2] == face[2]) {
				return true;
			}
		}
		return false;
	}
	template<typename V, typename E, typename F, typename H>
	void  MeshLib::MyMesh<V, E, F, H>::getface() {
		for (int i = 0; i < listOfVertex.size(); i++) {
			CHalfEdge *he = listOfVertex[i]->halfedge();
			CHalfEdge *current = he;
			do {
				vector<int> face;
				for (int i = 0; i < 3; i++) {
					face.push_back(current->target()->id());
					current = current->he_next();
				}
				
				Sort(face);
				if (!issameface(face)) {
					existFace.push_back(face);
					listOfFace.push_back(face);
				}
				if (current->he_sym() == NULL) {
					break;
				}
				face.clear();
				current = current->he_sym()->he_next()->he_next();
			} while (current != he);


		}
	}
	template<typename V, typename E, typename F, typename H>
	void  MeshLib::MyMesh<V, E, F, H>::save() {
		ofstream out(".\\data.m");
		for (int i = 0; i < listOfVertex.size(); i++) {
			out << "Vertex " + to_string(i + 1) + " " + to_string(listOfVertex[i]->point()[0]) +
				" " + to_string(listOfVertex[i]->point()[1]) + " " + to_string(listOfVertex[i]->point()[2]) << endl;
		}
		for (int i = 0; i < listOfFace.size(); i++) {
			out << "Face " + to_string(i + 1) + " " + to_string(listOfFace[i][0]) +
				" " + to_string(listOfFace[i][1]) + " " + to_string(listOfFace[i][2]) << endl;
		}
		out.flush();
		out.close();
	}
	template<typename V, typename E, typename F, typename H>
	void MeshLib::MyMesh<V, E, F, H>::output_mesh_info()
	{
		int nv = this->numVertices();
		int ne = this->numEdges();
		int nf = this->numFaces();

		std::cout << "#V=" << nv << "  ";
		std::cout << "#E=" << ne << "  ";
		std::cout << "#F=" << nf << "  ";

		int euler_char= nv - ne + nf;
		std::cout << "Euler's characteristic=" << euler_char << "  ";

		CBoundary boundary(this);
		std::vector<CLoop*> & loops = boundary.loops();
		int nb = loops.size();

		int genus = (2 - (euler_char + nb)) / 2;
		std::cout << "genus=" << genus << std::endl;
	}

	template<typename V, typename E, typename F, typename H>
	void MyMesh<V, E, F, H>::test_iterator()
	{
		for (MeshVertexIterator viter(this); !viter.end(); ++viter)
		{
			V * pV = *viter;
			// you can do something to the vertex here
			// ...

			for (VertexEdgeIterator veiter(pV); !veiter.end(); ++veiter)
			{
				E * pE = *veiter;
				// you can do something to the neighboring edges with CCW
				// ...
			}

			for (VertexFaceIterator vfiter(pV); !vfiter.end(); ++vfiter)
			{
				F * pF = *vfiter;
				// you can do something to the neighboring faces with CCW
				// ...
			}

			for (VertexInHalfedgeIterator vhiter(this, pV); !vhiter.end(); ++vhiter)
			{
				H * pH = *vhiter;
				// you can do something to the incoming halfedges with CCW
				// ...
			}
		}

		for (MeshEdgeIterator eiter(this); !eiter.end(); ++eiter)
		{
			E * pE = *eiter;
			// you can do something to the edge here
			// ...
		}

		for (MeshFaceIterator fiter(this); !fiter.end(); ++fiter)
		{
			F * pF = *fiter;
			// you can do something to the face here
			// ...
		}

		//there are some other iterators which you can find them in class MyMesh

		std::cout << "Iterators test OK.\n";
	}
}

#endif // !_MY_MESH_
