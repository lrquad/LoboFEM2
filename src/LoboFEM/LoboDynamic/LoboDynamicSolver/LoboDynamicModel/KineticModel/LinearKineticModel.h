#pragma once
#include "KineticModel.h"

namespace Lobo
{
class LinearKineticModel : public KineticModel
{
public:
    LinearKineticModel(LoboDynamicScene *scene_, LoboTetMesh *tetmesh_, HyperelasticModel *elastic_model_, ConstrainModel *constrain_model_, CollisionModel *collisionmodel_);
    ~LinearKineticModel();

    virtual void precompute();

    virtual void computeEnergySparse(Eigen::VectorXd*free_variables, double *energy, Eigen::VectorXd*jacobi, Eigen::SparseMatrix<double> *hessian, int computationflags);
    


private:

     Eigen::SparseMatrix<double> hessian_matrix;

};



} // namespace Lobo