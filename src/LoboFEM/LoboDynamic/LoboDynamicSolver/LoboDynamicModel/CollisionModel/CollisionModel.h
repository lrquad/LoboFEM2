#pragma once
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/DynamicModel.h"
#include "Utils/pugixml/pugixml.hpp"

namespace Lobo {
class LoboTetMesh;
class CollisionModel : public DynamicModel {
   private:
    /* data */
   public:
    CollisionModel(LoboTetMesh* tetmesh_);
    ~CollisionModel();

    virtual void runXMLscript(pugi::xml_node& xml_node);

    virtual void computeEnergySparse(Eigen::VectorXd* free_variables,
                                     double* energy, Eigen::VectorXd* jacobi,
                                     Eigen::SparseMatrix<double>* hessian,
                                     int computationflags);
    virtual void computeEnergyDense(Eigen::VectorXd* free_variables,
                                    double* energy, Eigen::VectorXd* jacobi,
                                    Eigen::MatrixXd* hessian,
                                    int computationflags){};

    double weight_stiffness;
    double friction_weight_stiffness;

    Eigen::VectorXd q_1;
    Eigen::VectorXd q_vel_1;


   protected:
    LoboTetMesh* tetmesh;
};
}  // namespace Lobo