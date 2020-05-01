

#include "LinearKineticModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/ElasticModel/HyperelasticModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/ConstrainModel/ConstrainModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/CollisionModel/CollisionModel.h"

Lobo::LinearKineticModel::LinearKineticModel(LoboDynamicScene *scene_, LoboTetMesh *tetmesh_, HyperelasticModel *elastic_model_, ConstrainModel *constrain_model_, CollisionModel *collisionmodel_) : KineticModel(scene_, tetmesh_, elastic_model_, constrain_model_, collisionmodel_)
{
}

Lobo::LinearKineticModel::~LinearKineticModel()
{
}


void Lobo::LinearKineticModel::precompute()
{
    Lobo::KineticModel::precompute();
    this->num_DOFs;
    Eigen::VectorXd zeroq(num_DOFs);
    Eigen::VectorXd zerof(num_DOFs);

    int flags_all = 0;
    flags_all |= Computeflags_second|Computeflags_reset;
    
    zeroq.setZero();
    double energy = 0;

    Lobo::KineticModel::getSparseTopoloty(hessian_matrix);
    Lobo::KineticModel::computeEnergySparse(&zeroq,&energy,&zerof,&hessian_matrix,flags_all);

    hyperelasticmodel->computeEnergySparse(&zeroq,&energy,&zerof,&stiffness_matrix,flags_all);
}

void Lobo::LinearKineticModel::computeEnergySparse(Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi, Eigen::SparseMatrix<double> *hessian, int computationflags)
{
    Eigen::VectorXd elastic_force(num_DOFs);
    if(computationflags&Computeflags_reset)
    {
        jacobi->setZero();
    }

    computationflags &= ~Computeflags_reset;

    // Eigen::VectorXd constrain_force(num_DOFs);
    if (constrainmodel->trigger)
    {
        constrainmodel->computeEnergySparse(free_variables, energy,
                                            jacobi, hessian,
                                            computationflags);
    }
    // double constrain_energy = *energy-elastic_energy;
    if(collisionmodel->trigger)
    {
        collisionmodel->q_1 = q_1;
        collisionmodel->computeEnergySparse(free_variables, energy,
                                            jacobi, hessian,
                                            computationflags);
    }

    if (computationflags & Computeflags_fisrt)
    {
        Eigen::VectorXd q_buffer = (*free_variables - q_1 - timestep * q_vel);

        //external_forces.setZero();
        //external_forces += gravity_force;

        elastic_force = stiffness_matrix**free_variables;
        *jacobi = elastic_force;

        Eigen::VectorXd all_ext = mass_matrix * (q_buffer) / (timestep * timestep) - external_forces;
        *jacobi += all_ext;
    }

    if (computationflags & Computeflags_second)
    {
        *hessian = hessian_matrix;
    }
}