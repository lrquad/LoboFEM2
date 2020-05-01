#pragma once
#include "LoboDynamic/LoboDynamic.h"

namespace Lobo
{
class ReducedKineticModel : public KineticModel
{
public:
    ReducedKineticModel(LoboDynamicScene *scene_, Eigen::MatrixXd *phi_, LoboTetMesh *tetmesh_,HyperelasticModel *elastic_model_, ConstrainModel *constrain_model_, CollisionModel *collisionmodel_);
    ~ReducedKineticModel();

    virtual void precompute();

    virtual void computeEnergyDense(Eigen::VectorXd*free_variables, double *energy, Eigen::VectorXd*jacobi, Eigen::MatrixXd*hessian, int computationflags);

    Eigen::MatrixXd phi; //linear subspace

protected:

    Eigen::VectorXd q;
    Eigen::VectorXd internalforce;
    int full_DoFs;

};
} // namespace Lobo