#include "ReducedKineticModel.h"

#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/ElasticModel/HyperelasticModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/ConstrainModel/ConstrainModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/CollisionModel/CollisionModel.h"

#include <time.h>

Lobo::ReducedKineticModel::ReducedKineticModel(LoboDynamicScene *scene_, Eigen::MatrixXd *phi_, LoboTetMesh *tetmesh_, HyperelasticModel *elastic_model_, ConstrainModel *constrain_model_, CollisionModel *collisionmodel_) : KineticModel(scene_, tetmesh_, elastic_model_, constrain_model_, collisionmodel_)
{
    phi = *phi_;

    num_DOFs = phi.cols();
    full_DoFs = phi.rows();

    external_forces.resize(full_DoFs);
    external_forces.setZero();

    q.resize(full_DoFs);
    q.setZero();

    internalforce.resize(full_DoFs);
    internalforce.setZero();
}

Lobo::ReducedKineticModel::~ReducedKineticModel()
{
}

void Lobo::ReducedKineticModel::precompute()
{

    Lobo::KineticModel::precompute();

    gravity_force.resize(full_DoFs);
    gravity_force.setZero();
    for (int i = 0; i < full_DoFs / 3; i++)
    {
        gravity_force.data()[i * 3 + 1] = -9.8;
    }

    gravity_force = mass_matrix * gravity_force;
}

void Lobo::ReducedKineticModel::computeEnergyDense(Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi, Eigen::MatrixXd *hessian, int computationflags)
{

    double elastic_energy = 0;
    q = phi * *free_variables;

    hyperelasticmodel->computeEnergySparse(&q, &elastic_energy, &internalforce, &stiffness_matrix, computationflags);

    double kinetic_energy;

    Eigen::VectorXd sn = q - q_1 - timestep * q_vel;
    Eigen::VectorXd M_sn = (mass_matrix)*sn;
    kinetic_energy = sn.dot(M_sn);
    kinetic_energy = kinetic_energy / (2.0 * timestep * timestep);

    double externalforce_energy;
    externalforce_energy = -external_forces.dot(q);

    if (computationflags & Computeflags_energy)
    {
        *energy = elastic_energy + kinetic_energy + externalforce_energy;
    }

    if (computationflags & (Computeflags_fisrt))
    {

        //comptue gradient
        *jacobi = phi.transpose() * M_sn / (timestep * timestep);

        *jacobi += phi.transpose() * internalforce;
        *jacobi -= phi.transpose() * external_forces;
        //std::cout << "compute f " << seconds << "s" << std::endl;
    }

    if (computationflags & (Computeflags_second))
    {

        *hessian = phi.transpose() * mass_matrix * phi / (timestep * timestep);
        *hessian += phi.transpose() * (stiffness_matrix)*phi;
    }
}