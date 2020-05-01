#include "ModalWarpingModel.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"

Lobo::ModalWarpingModel::ModalWarpingModel(LoboTetMesh *tetmesh_, std::vector<int> &warplist) : WarpModel(tetmesh_, warplist)
{
    modalrotationmatrix = new ModalRotationMatrix(tetmesh_);
    modal_rotation_sparsematrix = new Eigen::SparseMatrix<double>();
}

Lobo::ModalWarpingModel::~ModalWarpingModel()
{
    delete modal_rotation_sparsematrix;
    delete modalrotationmatrix;
}

void Lobo::ModalWarpingModel::precompute()
{
    modalrotationmatrix->computeModalRotationSparseMatrix_W(modalrotationmatrix->w_operator);
}
void Lobo::ModalWarpingModel::warp(Eigen::VectorXd &fullq)
{
    Eigen::VectorXd w = *(modalrotationmatrix->w_operator)*fullq;
	modalrotationmatrix->computeWarpingRotationVector_subnodes(fullq, w, warp_node_list);
}
void Lobo::ModalWarpingModel::warpForce(Eigen::VectorXd &force, Eigen::VectorXd &q, bool T)
{
    Eigen::VectorXd w = *(modalrotationmatrix->w_operator)*q;
	modalrotationmatrix->computeLocalOrientationVector(force, w, T);
}