#include"Simplification.h"
#include"Pair.h"
#include<algorithm>
void Simplification::initQ(CMyMesh *mesh) {
	for (CMyMesh::MeshVertexIterator viter(mesh); !viter.end(); ++viter)
	{
		CVertex * pV = *viter;
		pV->setQ(util::getQ(pV));
	}
}
void Simplification::getAllValidPair(CMyMesh *mesh) {
	for (CMyMesh::MeshEdgeIterator eiter(mesh); !eiter.end(); ++eiter)
	{
		CEdge * pE = *eiter;
		CVertex *a,*b;
		a = pE->halfedge(0)->target();
		b = pE->halfedge(0)->he_next()->he_next()->target();
		if (!(pE->boundary())) {
			if (!(pE->halfedge(0)->he_next()->edge()->boundary()) && !(pE->halfedge(0)->he_prev()->edge()->boundary()) && !(pE->halfedge(1)->he_next()->edge()->boundary()) && !(pE->halfedge(1)->he_prev()->edge()->boundary())) {
				Pair pair(a, b);
				allPair.push_back(pair);
			}
		}
	}
}
void Simplification::getCost(Pair &p) {
	Matrix4d addQ = p.p1->getQ() + p.p2->getQ();
	/*cout << addQ << endl;
	getchar();*/
	Matrix4d addQ1;
	for (int k = 0; k < 3; k++) {
		for (int j = 0; j < 4; j++) {
			addQ1(k, j) = addQ(k, j);
		}
	}
	addQ1(3, 0) = 0;
	addQ1(3, 1) = 0;
	addQ1(3, 2) = 0;
	addQ1(3, 3) = 1;
	Vector4d a;
	a << 0, 0, 0, 1;
	Vector4d result,p1,p2;
	p1 << p.p1->point()[0], p.p1->point()[1], p.p1->point()[2], 1;
	p2 << p.p2->point()[0], p.p2->point()[1], p.p2->point()[2], 1;
	if (isnan(addQ1.inverse()(0))) {
		double maxk=0, maxcost=0;
		double cost;
		for (double k = 0; k <= 1; k = k + 0.1) {
			result = (1 - k)*p1 + k*p2;
			cost= (result.transpose()*addQ*result)(0);
			if (cost < maxcost) {
				maxk = k;
			}
		}
		result = (1 - maxk)*p1 + maxk*p2;
	}
	else {
		result = addQ1.inverse()*a;
	}
	
	for (int k = 0; k < 3; k++) {
		p.v[k] = result(k);
	}
	p.Cost = (result.transpose()*addQ*result)(0);
	
}
void Simplification::getAllCost() {
	for (int i = 0; i < allPair.size(); i++) {
		getCost(allPair[i]);
	}
}
bool cmp1(Pair a, Pair b) {
	return a.Cost < b.Cost;
}

void Simplification::BuildPairHeap() {
	make_heap(allPair.begin(), allPair.end(), cmp1);
}
void Simplification::contraction(CMyMesh *mesh,Pair p) {
	CMyVertex *v = mesh->idVertex(p.p2->id());
	p.p1->point() = p.v;
	p.p1->setQ(p.p1->getQ() + p.p2->getQ());
	//CMyVertex *v;
	//记录半边
	CHalfEdge*pHe3 = NULL, *pHe4 = NULL, *pHe5 = NULL, *pHe6 = NULL,*pH1=NULL,*pH2=NULL;
		//if (!(e->boundary())) {
		//	if(!(e->halfedge(0)->he_next()->edge()->boundary())&& !(e->halfedge(0)->he_prev()->edge()->boundary()) && !(e->halfedge(1)->he_next()->edge()->boundary())&& !(e->halfedge(1)->he_prev()->edge()->boundary())){
				//for (int i = 0; i < allPair.size(); i++) {
				//	if (allPair[i].p1->id() == p.p1->id()) {
				//		allPair[i].p1->point() = p.v;
				//		allPair[i].p1->setQ(p.p1->getQ());
				//		getCost(allPair[i]);
				//	}
				//	if (allPair[i].p2->id() == p.p1->id()) {
				//		allPair[i].p2->point() = p.v;
				//		allPair[i].p2->setQ(p.p1->getQ());
				//		getCost(allPair[i]);
				//	}
				//	if (allPair[i].p1->id() == p.p2->id()) {
				//		allPair[i].p1 = p.p1;
				//		getCost(allPair[i]);
				//	}
				//	if (allPair[i].p2->id() == p.p2->id()) {
				//		allPair[i].p2 = p.p1;
				//		getCost(allPair[i]);
				//	}
				//}

				//for (int i = 0; i < allPair.size(); i++) {
				//	for (int j = i + 1; j < allPair.size(); j++) {
				//		if ((allPair[i].p1->id() == allPair[j].p1->id()&&allPair[i].p2->id() == allPair[j].p2->id()) || (allPair[i].p1->id() == allPair[j].p2->id() &&allPair[i].p2->id() == allPair[j].p1->id())) {
				//			
				//				allPair.erase(allPair.begin() + i);

				//				//std::cout << "delete" << allPair[i].e->halfedge(0)->source()->id() <<" "<< allPair[i].e->halfedge(0)->target()->id() << endl;
				//				//std::cout << "get" << allPair[j].e->halfedge(0)->source()->id() <<" "<< allPair[j].e->halfedge(0)->target()->id() << endl;

				//				BuildPairHeap();
				//			}
				//			
				//		}
				//	
				//}
				//bool *same = new bool[allPair.size()];
				//memset(same, 0, sizeof(same));
				//for (int i = 0; i < allPair.size(); i++) {
				//	if (allPair[i].p1->point() == p.v || allPair[i].p2->point() == p.v) {
				//		same[i] = 1;
				//		//cout << "i=" << allPair[i].p1->id() << " " << allPair[i].p2->id() << endl;
				//	}
				//}
				//for (int i = 0; i < allPair.size(); i++) {
				//	if (same[i] == 1) {
				//		getCost(allPair[i]);
				//	}
				//}
				//BuildPairHeap();
			//找到p1,p2所在的边e
			CEdge* e = util::locateEdge(mesh,p.p1, p.p2);
			if ((e!=NULL)&&!(e->boundary()) && (e->halfedge(0)->he_next()->he_sym()!=NULL) && (e->halfedge(0)->he_prev()->he_sym() != NULL) && (e->halfedge(1)->he_next()->he_sym() != NULL) && (e->halfedge(1)->he_prev()->he_sym() != NULL)) {
				pH1 = e->halfedge(0);
				pH2 = e->halfedge(1);
				//记录不删的半边
				if (e->halfedge(0)->target()->id() == p.p2->id()) {
					pHe3 = e->halfedge(0)->he_next()->he_sym();	
					pHe4 = e->halfedge(0)->he_prev()->he_sym();
					pHe6 = e->halfedge(1)->he_next()->he_sym();
					pHe5 = e->halfedge(1)->he_prev()->he_sym();
				}
				else {
					pHe3 = e->halfedge(1)->he_next()->he_sym();
					pHe4 = e->halfedge(1)->he_prev()->he_sym();
					pHe5 = e->halfedge(0)->he_prev()->he_sym();
					pHe6 = e->halfedge(0)->he_next()->he_sym();
				}
				if (pHe4 == NULL) {
					cout << "PHe4 is boundary3" << endl;
					getchar();
				}
				//删除面
				if (pH1 != NULL) {
					mesh->deleteFace((CMyFace*)pH1->face());
				}
				if (pH2 != NULL) {
					mesh->deleteFace((CMyFace*)pH2->face());
				}
				//删除边
				util::deleteEdge(mesh, pHe3->edge());
				util::deleteEdge(mesh, pHe5->edge());
				//链接对偶和边
				pHe4->he_sym() = pHe3;
				pHe3->edge() = pHe4->edge();
				pHe6->he_sym() = pHe5;
				pHe5->edge() = pHe6->edge();
				//改变所有和p2有关的半边走向
				for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
					CHalfEdge *pH = *vieiter;
					pH->target() = p.p1;
				}
				for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
					CHalfEdge *pH = *vieiter;
					pH->source() = p.p1;
				}
				//删除p2点
				mesh->vertices().remove((CMyVertex *)p.p2);
				//mesh->vertices().erase(find(mesh->vertices().begin(), mesh->vertices().end(), p.p2));
				cout << "merge success" << endl;
			}
	//		pH1 = e->halfedge(0);
	//		pH2 = e->halfedge(1);
	//		if ((e != NULL) && !(e->boundary())) {
	//			
	//			if (e->halfedge(0)->target()->id() == p.p2->id()) {
	//				if (e->halfedge(0)->he_next()->edge()->boundary()&& e->halfedge(1)->he_prev()->edge()->boundary()) {//上下两面都是边界，删面即可
	//					
	//					mesh->deleteFace((CMyFace*)pH1->face());
	//					mesh->deleteFace((CMyFace*)pH2->face());
	//					//util::deleteEdge(mesh, e->halfedge(0)->he_next()->edge())
	//				}
	//				else if (e->halfedge(0)->he_next()->edge()->boundary() && !(e->halfedge(1)->he_prev()->edge()->boundary())) {//上面边界，下面不是，删面，改走向，删边
	//					
	//					
	//					if (e->halfedge(1)->he_next()->he_sym() != NULL) {
	//						pHe3 = e->halfedge(1)->he_prev()->he_sym();
	//						pHe4 = e->halfedge(1)->he_next()->he_sym();
	//						pHe4->he_sym() = pHe3;
	//						pHe3->edge() = pHe4->edge();

	//						for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->target() = p.p1;
	//						}
	//						for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->source() = p.p1;
	//						}
	//					}
	//					mesh->deleteFace((CMyFace*)pH1->face());
	//					mesh->deleteFace((CMyFace*)pH2->face());
	//					util::deleteEdge(mesh, e->halfedge(1)->he_prev()->edge());
	//				}
	//				else if (!(e->halfedge(0)->he_next()->edge()->boundary()) && e->halfedge(1)->he_prev()->edge()->boundary()) {
	//					
	//					if (e->halfedge(0)->he_prev()->he_sym() != NULL) {
	//						pHe3 = e->halfedge(0)->he_next()->he_sym();
	//						pHe4 = e->halfedge(0)->he_prev()->he_sym();
	//						pHe4->he_sym() = pHe3;
	//						pHe3->edge() = pHe4->edge();

	//						for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->target() = p.p1;
	//						}
	//						for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->source() = p.p1;
	//						}
	//					}
	//					mesh->deleteFace((CMyFace*)pH1->face());
	//					mesh->deleteFace((CMyFace*)pH2->face());
	//					util::deleteEdge(mesh, e->halfedge(0)->he_next()->edge());
	//				}
	//				else {
	//					if (e->halfedge(0)->he_prev()->he_sym() != NULL&&e->halfedge(1)->he_next()->he_sym() != NULL) {
	//						pHe3 = e->halfedge(0)->he_next()->he_sym();
	//						pHe4 = e->halfedge(0)->he_prev()->he_sym();
	//						pHe6 = e->halfedge(1)->he_next()->he_sym();
	//						pHe5 = e->halfedge(1)->he_prev()->he_sym();
	//						mesh->deleteFace((CMyFace*)pH1->face());
	//						mesh->deleteFace((CMyFace*)pH2->face());
	//						util::deleteEdge(mesh, pHe3->edge());
	//						util::deleteEdge(mesh, pHe5->edge());
	//						//链接对偶和边
	//						pHe4->he_sym() = pHe3;
	//						pHe3->edge() = pHe4->edge();
	//						pHe6->he_sym() = pHe5;
	//						pHe5->edge() = pHe6->edge();
	//						//改变所有和p2有关的半边走向
	//						for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->target() = p.p1;
	//						}
	//						for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->source() = p.p1;
	//						}
	//					}
	//				}
	//			}
	//			else {
	//				if (e->halfedge(0)->he_prev()->edge()->boundary() && e->halfedge(1)->he_next()->edge()->boundary()) {//上下两面都是边界，删面即可

	//					mesh->deleteFace((CMyFace*)pH1->face());
	//					mesh->deleteFace((CMyFace*)pH2->face());
	//					//util::deleteEdge(mesh, e->halfedge(0)->he_next()->edge())
	//				}
	//				else if (e->halfedge(0)->he_prev()->edge()->boundary() && !(e->halfedge(1)->he_next()->edge()->boundary())) {//上面边界，下面不是，删面，改走向，删边


	//					if (e->halfedge(1)->he_prev()->he_sym() != NULL) {
	//						pHe4 = e->halfedge(1)->he_prev()->he_sym();
	//						pHe3 = e->halfedge(1)->he_next()->he_sym();
	//						pHe4->he_sym() = pHe3;
	//						pHe3->edge() = pHe4->edge();

	//						for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->target() = p.p1;
	//						}
	//						for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->source() = p.p1;
	//						}
	//					}
	//					mesh->deleteFace((CMyFace*)pH1->face());
	//					mesh->deleteFace((CMyFace*)pH2->face());
	//					util::deleteEdge(mesh, e->halfedge(1)->he_next()->edge());
	//				}
	//				else if (!(e->halfedge(0)->he_prev()->edge()->boundary()) && e->halfedge(1)->he_next()->edge()->boundary()) {

	//					if (e->halfedge(0)->he_next()->he_sym() != NULL) {
	//						pHe4 = e->halfedge(0)->he_next()->he_sym();
	//						pHe3 = e->halfedge(0)->he_prev()->he_sym();
	//						pHe4->he_sym() = pHe3;
	//						pHe3->edge() = pHe4->edge();

	//						for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->target() = p.p1;
	//						}
	//						for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->source() = p.p1;
	//						}
	//					}
	//					mesh->deleteFace((CMyFace*)pH1->face());
	//					mesh->deleteFace((CMyFace*)pH2->face());
	//					util::deleteEdge(mesh, e->halfedge(0)->he_prev()->edge());
	//				}
	//				else {
	//					if (e->halfedge(0)->he_next()->he_sym() != NULL&&e->halfedge(1)->he_prev()->he_sym() != NULL) {
	//						pHe3 = e->halfedge(1)->he_next()->he_sym();
	//						pHe4 = e->halfedge(1)->he_prev()->he_sym();
	//						pHe5 = e->halfedge(0)->he_prev()->he_sym();
	//						pHe6 = e->halfedge(0)->he_next()->he_sym();
	//						mesh->deleteFace((CMyFace*)pH1->face());
	//						mesh->deleteFace((CMyFace*)pH2->face());
	//						util::deleteEdge(mesh, pHe3->edge());
	//						util::deleteEdge(mesh, pHe5->edge());
	//						//链接对偶和边
	//						pHe4->he_sym() = pHe3;
	//						pHe3->edge() = pHe4->edge();
	//						pHe6->he_sym() = pHe5;
	//						pHe5->edge() = pHe6->edge();
	//						//改变所有和p2有关的半边走向
	//						for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->target() = p.p1;
	//						}
	//						for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//							CHalfEdge *pH = *vieiter;
	//							pH->source() = p.p1;
	//						}
	//					}
	//				}
	//			}
	//		}
	//		else if (e != NULL&&e->boundary()) {
	//			if (pH1->he_next()->edge()->boundary() || pH1->he_prev()->edge()->boundary()) {
	//				mesh->deleteFace((CMyFace*)pH1->face());
	//			}
	//			else {
	//				pHe4 = pH1->he_next()->he_sym();
	//				pHe3= pH1->he_prev()->he_sym();
	//				mesh->deleteFace((CMyFace*)pH1->face());;
	//				pHe4->he_sym() = pHe3;
	//				pHe3->edge() = pHe4->edge();
	//				for (CMyMesh::VertexInHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//					CHalfEdge *pH = *vieiter;
	//					pH->target() = p.p1;
	//				}
	//				for (CMyMesh::VertexOutHalfedgeIterator vieiter(mesh, v); !vieiter.end(); ++vieiter) {
	//					CHalfEdge *pH = *vieiter;
	//					pH->source() = p.p1;
	//				}
	//			}
	//		}
	//mesh->vertices().remove((CMyVertex *)p.p2);
	std::cout << "allPair:" << allPair.size() << endl;
}
void Simplification::simplificate(CMyMesh *mesh,double ratio) {
	initQ(mesh);
	getAllValidPair(mesh);
	getAllCost();
	//BuildPairHeap();
	double a = mesh->numFaces();
	while (mesh->numFaces()>a*ratio) {
		//BuildPairHeap();
		sort(allPair.begin(), allPair.end(), cmp1);
		Pair p = allPair[0];
		cout << "merge:"  << p.p1->id() << " " << p.p2->id() << endl;
		//getchar();
		cout << p.Cost << endl;
		//getchar();
		//pop_heap(allPair.begin(), allPair.end(), cmp1);
		//cout<<"pop_one:" << allPair[allPair.size()-1].p1->id() << " " << allPair[allPair.size() - 1].p2->id() << endl;
		//allPair.erase(allPair.begin());
		//BuildPairHeap();
		contraction(mesh,p);
		allPair.clear();
		getAllValidPair(mesh);
		getAllCost();
		//BuildPairHeap();
	}
	cout << mesh->numFaces() << endl;
}
