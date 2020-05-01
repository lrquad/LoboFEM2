#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

namespace Lobo
{
class LoboTetMesh;

class ModalRotationMatrix
{
public:
    ModalRotationMatrix(LoboTetMesh *tetmesh_);
    ~ModalRotationMatrix();

    virtual void computeModalRotationSparseMatrix_W(Eigen::SparseMatrix<double> *W);

    virtual void computeWarpingRotationVector_subnodes(Eigen::VectorXd &u, Eigen::VectorXd &w, std::vector<int> &nodelist);

    virtual void computeLocalOrientationVector(Eigen::VectorXd &f, const Eigen::VectorXd &w, bool transpose = true);

    Eigen::SparseMatrix<double> *w_operator;

protected:
    LoboTetMesh *tetmesh;
    std::vector<std::vector<int>> node_elements;
};

} // namespace Lobo