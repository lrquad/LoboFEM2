#include "BVHCollisionDetector.h"
#include "BVHTriModel.h"
#include "Functions/computeTriangle.h"
#include <iostream>
using namespace Eigen;

BVHCollisionDetector::BVHCollisionDetector(Lobo::LoboMesh* trimesh)
{
	this->trimesh = trimesh;
	setIsstatic(true);
	setSelfCollisionTest(false);
	initCollisionShape();

}

BVHCollisionDetector::~BVHCollisionDetector()
{
	delete bvhTriModel;
}


void BVHCollisionDetector::initCollisionShape()
{
	bvhTriModel = new BVHTriModel(trimesh);
	bvhTriModel->BuildBVH();
}

void BVHCollisionDetector::updateCollisionShape()
{
	bvhTriModel->updateDeform();
	bvhTriModel->RefitBVH();
}

void BVHCollisionDetector::collideWith(BVHCollisionDetector* objtB)
{
	BVHTriModel* other = objtB->getBVHmodel();
	bvhTriModel->Collide(other);
	if (bvhTriModel->NumContact() != 0)
	{
		for (int i = 0;i < bvhTriModel->NumContact();i++)
		{
			//ShapeOpModel::s_rets;
			unsigned int triid1, triid2;
			bvhTriModel->GetContact(i, triid1, triid2);
			computeCollideInfo((int)triid1, (int)triid2, objtB);
		}
	}
}

void BVHCollisionDetector::selfCollision()
{
	bvhTriModel->SelfCollide();

	if (bvhTriModel->NumContact() != 0)
	{
		for (int i = 0;i < bvhTriModel->NumContact();i++)
		{
			//ShapeOpModel::s_rets;
			unsigned int triid1, triid2;
			bvhTriModel->GetContact(i, triid1, triid2);
			computeCollideInfo((int)triid1, (int)triid2, this);
		}
	}
}

void BVHCollisionDetector::addNewCollitionInfo(int triid, int nodeid, double displacemenet, Eigen::Vector3d norms)
{
	CollideInfo newcollideinfo = { triid, nodeid, displacemenet, norms };
	collitionInfo.push_back(newcollideinfo);
}

void BVHCollisionDetector::clearCollitionInfo()
{
	collitionInfo.clear();
}

void BVHCollisionDetector::computeCollideInfo(int tri1, int tri2, BVHCollisionDetector* objB)
{
	BVHTriModel* other = objB->getBVHmodel();
	int tri1_node[3];
	int tri2_node[3];

	tri1_node[0] = bvhTriModel->getTriAngleFaceNode(tri1, 0);
	tri1_node[1] = bvhTriModel->getTriAngleFaceNode(tri1, 1);
	tri1_node[2] = bvhTriModel->getTriAngleFaceNode(tri1, 2);

	tri2_node[0] = other->getTriAngleFaceNode(tri2, 0);
	tri2_node[1] = other->getTriAngleFaceNode(tri2, 1);
	tri2_node[2] = other->getTriAngleFaceNode(tri2, 2);

	Vector3d tri1_nodePosition[3];
	Vector3d tri2_nodePosition[3];
	tri1_nodePosition[0] = bvhTriModel->getTriNode(tri1_node[0]);
	tri1_nodePosition[1] = bvhTriModel->getTriNode(tri1_node[1]);
	tri1_nodePosition[2] = bvhTriModel->getTriNode(tri1_node[2]);

	tri2_nodePosition[0] = other->getTriNode(tri2_node[0]);
	tri2_nodePosition[1] = other->getTriNode(tri2_node[1]);
	tri2_nodePosition[2] = other->getTriNode(tri2_node[2]);

	Vector3d normal1, normal2;
	normal1 = bvhTriModel->getTriNorm(tri1);
	normal2 = other->getTriNorm(tri2);
	//computeTriangleNorm(tri1_nodePosition[0], tri1_nodePosition[1], tri1_nodePosition[2], normal1);
	//computeTriangleNorm(tri2_nodePosition[0], tri2_nodePosition[1], tri2_nodePosition[2], normal2);

	for (int i = 0; i < 3; i++)
	{
		double distance_Nodei_Trai2 = Lobo::computeDistancePointToTriangle(tri2_nodePosition[0], tri2_nodePosition[1], tri2_nodePosition[2], normal2, tri1_nodePosition[i]);
		bool intriangle = Lobo::checkPointTriangleProjection(tri2_nodePosition[0], tri2_nodePosition[1], tri2_nodePosition[2],  tri1_nodePosition[i]);

		if (distance_Nodei_Trai2 < 0)
		{
			this->addNewCollitionInfo(tri1, tri1_node[i], distance_Nodei_Trai2, normal2);
		}

		double distance_Nodei_Trai1 = Lobo::computeDistancePointToTriangle(tri1_nodePosition[0], tri1_nodePosition[1], tri1_nodePosition[2], normal1, tri2_nodePosition[i]);
		intriangle = Lobo::checkPointTriangleProjection(tri1_nodePosition[0], tri1_nodePosition[1], tri1_nodePosition[2],  tri2_nodePosition[i]);

		if (distance_Nodei_Trai1 < 0)
		{
			objB->addNewCollitionInfo(tri2, tri2_node[i], distance_Nodei_Trai1, normal1);
		}
	}
}