#pragma once
#include "LoboVolumtricMesh/LoboTetMesh.h"
#include "LoboDynamic/LoboDynamicScene.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/DynamicModel.h"
#include "Utils/SparseMatrix/SparseMatrixTopology.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace Lobo
{

class HyperelasticModel;
class ConstrainModel;
class CollisionModel;

class KineticModel : public DynamicModel
{
private:
    /* data */
public:
    KineticModel(LoboDynamicScene *scene_, LoboTetMesh *tetmesh_,HyperelasticModel* elastic_model_,ConstrainModel* constrain_model_,CollisionModel* collisionmodel_);
    ~KineticModel();

    //precomptue final sparse matrix topology
    virtual void precompute();

    virtual void computeFiledForce(Eigen::Vector3d &force);

    virtual void runXMLscript(pugi::xml_node &xml_node);

    virtual void setKineticStatus(Eigen::VectorXd &q_vel_,Eigen::VectorXd &q_1_);

    virtual void computeEnergySparse(Eigen::VectorXd*free_variables, double *energy, Eigen::VectorXd*jacobi, Eigen::SparseMatrix<double> *hessian, int computationflags);
    virtual void computeEnergyDense(Eigen::VectorXd*free_variables, double *energy, Eigen::VectorXd*jacobi, Eigen::MatrixXd*hessian, int computationflags){};

    virtual void getSparseTopoloty(Eigen::SparseMatrix<double> &spmatrix);

    void setTimeStep(double v){timestep = v;}

    Eigen::VectorXd external_forces;
    Eigen::VectorXd gravity_force;

protected:

    virtual void computeAccelerationIndices(SparseMatrixTopologyTYPE<double> *sparsetopology);

    LoboDynamicScene *scene;
    LoboTetMesh *tetmesh;

    HyperelasticModel* hyperelasticmodel;
    ConstrainModel* constrainmodel;
    CollisionModel* collisionmodel;

    Eigen::VectorXd q_vel;
    Eigen::VectorXd q_1;
    double timestep;

    Eigen::SparseMatrix<double> mass_matrix;
    Eigen::SparseMatrix<double> stiffness_matrix_topology;
    Eigen::SparseMatrix<double> stiffness_matrix;
   
};
} // namespace Lobo