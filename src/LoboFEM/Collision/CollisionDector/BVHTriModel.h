#pragma once
#include "Collision/ShapeOpModel.h"
#include <Eigen/Dense>
#include "LoboMesh/LoboMesh.h"


class BVHTriModel:public ShapeOpModel
{
public:
	BVHTriModel(Lobo::LoboMesh* trimesh_);
	~BVHTriModel();

	virtual void updateDeform();

	int getTriAngleFaceNode(int tri1, int index);
	Eigen::Vector3d getTriNode(int nodeid);
	Eigen::Vector3d getTriNorm(int triid);

protected:
	Lobo::LoboMesh* trimesh;
	Eigen::VectorXf trimeshposition;
	Eigen::VectorXi trimeshindices;

};

