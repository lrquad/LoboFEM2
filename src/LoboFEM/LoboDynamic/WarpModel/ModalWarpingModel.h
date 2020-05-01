#pragma once

#include "WarpModel.h"
#include "ModalRotationMatrix.h"

namespace Lobo
{
class ModalWarpingModel : public WarpModel
{
public:
    ModalWarpingModel(LoboTetMesh *tetmesh_, std::vector<int> &warplist);
    ~ModalWarpingModel();

    virtual void precompute();
    virtual void warp(Eigen::VectorXd &fullq);
    virtual void warpForce(Eigen::VectorXd &force, Eigen::VectorXd &q, bool T = true);

    ModalRotationMatrix *modalrotationmatrix;
    Eigen::SparseMatrix<double> *modal_rotation_sparsematrix;
};
} // namespace Lobo