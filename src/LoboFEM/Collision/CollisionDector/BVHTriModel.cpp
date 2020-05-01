#include "BVHTriModel.h"
#include "Collision/vec4d.h"
#include <iostream>
using namespace Eigen;

BVHTriModel::BVHTriModel(Lobo::LoboMesh* trimesh_)
{
	this->trimesh = trimesh_;
	int numVertex = trimesh->attrib.vertices.size()/3;
	trimeshposition.resize(numVertex*3);
	trimeshindices.resize(trimesh->num_faces*3);
	
	trimesh->getCurVertices(trimeshposition.data());
	trimesh->getFaceIndices(trimeshindices.data());
	//trimeshposition = trimesh->getCurrentPositionFloat();
	//trimeshindices = trimesh->getIndices();
	
	this->_num_vtx = trimeshposition.size() / 3;
	this->_num_tri = trimeshindices.size() / 3;

	_tris = new unsigned int[_num_tri * 3];
	for (int i = 0;i < _num_tri * 3;i++)
	{
		_tris[i] = (unsigned int)trimeshindices.data()[i];
	}
	_vtxs = new vec3d[_num_vtx];

	for (int i = 0;i < _num_vtx;i++)
	{
		_vtxs[i].x = trimeshposition.data()[i * 3 + 0];
		_vtxs[i].y = trimeshposition.data()[i * 3 + 1];
		_vtxs[i].z = trimeshposition.data()[i * 3 + 2];
	}

	_colors = new unsigned char[_num_vtx * 3];
	_nrms = new vec3f[_num_vtx];
	_tri_nrms = new vec3f[_num_tri];

	this->UpdateNorm();
}

BVHTriModel::~BVHTriModel()
{
	delete[] _tris;

	delete[] _vtxs;
	delete[] _colors;
	delete[] _nrms;
	delete[] _tri_nrms;
}

void BVHTriModel::updateDeform()
{
	//trimeshposition = trimesh->getCurrentPositionFloat();
	trimesh->getCurVertices(trimeshposition.data());

	for (int i = 0;i < _num_vtx;i++)
	{
		_vtxs[i].x = trimeshposition.data()[i * 3 + 0];
		_vtxs[i].y = trimeshposition.data()[i * 3 + 1];
		_vtxs[i].z = trimeshposition.data()[i * 3 + 2];
	}
	this->UpdateNorm();
}

int BVHTriModel::getTriAngleFaceNode(int tri1, int index)
{
	return trimeshindices.data()[tri1 * 3 + index];
}

Eigen::Vector3d BVHTriModel::getTriNode(int nodeid)
{
	Vector3d node(trimeshposition.data()[nodeid * 3 + 0],
		trimeshposition.data()[nodeid * 3 + 1],
		trimeshposition.data()[nodeid * 3 + 2]
	);
	return node;
}

Eigen::Vector3d BVHTriModel::getTriNorm(int triid)
{
	Vector3d norm_(_tri_nrms[triid].v[0],
		_tri_nrms[triid].v[1],
		_tri_nrms[triid].v[2]
	);

	return norm_.normalized();
}
