#pragma once
#include "LoboDynamic/LoboDynamic.h"

namespace Lobo {
class GraspContactModel;
class GraspKineticModel : public KineticModel {
   public:
    GraspKineticModel(LoboDynamicScene* scene_, LoboTetMesh* tetmesh_,
                      HyperelasticModel* elastic_model_,
                      ConstrainModel* constrain_model_,
                      CollisionModel* collisionmodel_,
                      GraspContactModel* graspmodel_);
    ~GraspKineticModel();

    virtual void precompute();
    virtual void computeEnergySparse(Eigen::VectorXd*free_variables, double *energy, Eigen::VectorXd*jacobi, Eigen::SparseMatrix<double> *hessian, int computationflags);


    

   protected:
    GraspContactModel* graspmodel;
};

}  // namespace Lobo
