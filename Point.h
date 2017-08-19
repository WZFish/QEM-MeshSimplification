#pragma once
#include<Eigen/Dense>
#include<vector>
using namespace std;
using namespace Eigen;
class Point {
public:
	int id;
	Vector3d pos;
	vector<int>neighbor;
	Matrix4d Q;
	Point operator=(Point &_p);
	bool hasNeighbor(int Pointid);
	void addNeighbor(int Pointid);
	void CalculateQ(Point *p);

};
