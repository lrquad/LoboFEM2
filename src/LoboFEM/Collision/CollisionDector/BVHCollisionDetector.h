#pragma once
#include <Eigen/Dense>
#include <vector>
#include "LoboMesh/LoboMesh.h"

class BVHTriModel;

struct CollideInfo
{
	int collideTri;
	int nodeid;
	double displacement;
	Eigen::Vector3d norms;
};

class BVHCollisionDetector
{
public:
	BVHCollisionDetector(Lobo::LoboMesh* trimesh);
	~BVHCollisionDetector();

	virtual void initCollisionShape();
	virtual void updateCollisionShape();

	virtual void collideWith(BVHCollisionDetector* objtB);
	virtual void selfCollision();

	BVHTriModel* getBVHmodel()const { return bvhTriModel; };

	void addNewCollitionInfo(int triid, int nodeid, double displacemenet, Eigen::Vector3d norms);
	void clearCollitionInfo();

	int getNumCollideInfoSize() { return collitionInfo.size(); };
	CollideInfo getCollideInfo(int id) { return collitionInfo[id]; };

	bool getIsstatic() const { return isstatic; }
	void setIsstatic(bool val) { isstatic = val; }
	bool getSelfCollisionTest() const { return selfCollisionTest; }
	void setSelfCollisionTest(bool val) { selfCollisionTest = val; }

protected:

	void computeCollideInfo(int tri1, int tri2, BVHCollisionDetector* objB);
	std::vector<CollideInfo> collitionInfo;
	BVHTriModel* bvhTriModel;
	Lobo::LoboMesh* trimesh;

	bool isstatic;
	bool selfCollisionTest;

};

