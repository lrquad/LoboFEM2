#include "GraspKineticModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/ElasticModel/HyperelasticModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/ConstrainModel/ConstrainModel.h"
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/CollisionModel/CollisionModel.h"
#include "GraspContactModel.h"

Lobo::GraspKineticModel::GraspKineticModel(LoboDynamicScene *scene_,
                                 LoboTetMesh *tetmesh_,
                                 HyperelasticModel *elastic_model_,
                                 ConstrainModel *constrain_model_,
                                 CollisionModel *collisionmodel_,
                                 GraspContactModel* graspmodel_):
                                 KineticModel(scene_,tetmesh_,elastic_model_,constrain_model_,collisionmodel_)
{
    this->graspmodel = graspmodel_;
}

Lobo::GraspKineticModel::~GraspKineticModel()
{

}

void Lobo::GraspKineticModel::precompute()
{
    tetmesh->computeDiagMassMatrix(&mass_matrix);
    hyperelasticmodel->computeStiffnessMatrixTopology(
        &stiffness_matrix_topology);
    stiffness_matrix = stiffness_matrix_topology;
    // in this model, the final system matrix has the same topology as stiffness
    // matrix
    SparseMatrixTopologyTYPE<double> sparseMatrixTopology(
        &stiffness_matrix_topology);

    computeAccelerationIndices(&sparseMatrixTopology);

    hyperelasticmodel->precompute();
    hyperelasticmodel->setAccelerationIndices(row_, column_);
    constrainmodel->setAccelerationDiagIndices(diagonal_);
    collisionmodel->setAccelerationDiagIndices(diagonal_);
    graspmodel->setAccelerationDiagIndices(diagonal_);

    //precomptue gravity force
    gravity_force.resize(num_DOFs);
    gravity_force.setZero();


    for (int i = 0; i < num_DOFs / 3; i++)
    {
        gravity_force.data()[i * 3 + 1] = -9.8;
    }

    if (mass_matrix.cols() == gravity_force.rows())
        gravity_force = mass_matrix * gravity_force;

}

void Lobo::GraspKineticModel::computeEnergySparse(
    Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi,
    Eigen::SparseMatrix<double> *hessian, int computationflags)
{
    Eigen::VectorXd elastic_force(num_DOFs);
    //jacobi = - internal force
    hyperelasticmodel->computeEnergySparse(free_variables, energy,
                                           jacobi, hessian,
                                           computationflags);

    //test elastic first
    double elastic_energy = *energy;
    //just add value to stiffness_matrix
    computationflags &= ~Computeflags_reset;

    // Eigen::VectorXd constrain_force(num_DOFs);
    if (constrainmodel->trigger)
    {
        constrainmodel->computeEnergySparse(free_variables, energy,
                                            jacobi, hessian,
                                            computationflags);
    }

    // double constrain_energy = *energy-elastic_energy;
    if (collisionmodel->trigger)
    {
        collisionmodel->q_1 = q_1;
        collisionmodel->computeEnergySparse(free_variables, energy,
                                            jacobi, hessian,
                                            computationflags);
    }

    if(graspmodel->trigger)
    {
        graspmodel->q_1 = q_1;
        graspmodel->computeEnergySparse(free_variables, energy,
                                            jacobi, hessian,
                                            computationflags);
    }

    //kinetic parts
    Eigen::VectorXd q_buffer = (*free_variables - q_1 - timestep * q_vel);

    //
    //external_forces.setZero();
    //external_forces += gravity_force;

    //energy
    if (computationflags & Computeflags_energy)
    {
        double inv_t = 1.0 / timestep;
        Eigen::VectorXd e = q_buffer.transpose() * mass_matrix * q_buffer;
        *energy += e.data()[0] * inv_t * inv_t * 0.5;

        for (int k = 0; k < num_DOFs; k++)
        {
            //energy += diag_mass_matrix[i] * inv_t*inv_t * q_buffer*q_buffer*0.5;
            *energy += -external_forces[k] * free_variables->data()[k];
        }
    }

    //jacobi
    if (computationflags & Computeflags_fisrt)
    {
        Eigen::VectorXd all_ext = mass_matrix * (q_buffer) / (timestep * timestep) - external_forces;
        *jacobi += all_ext;
    }

    if (computationflags & Computeflags_second)
    {
        //hessian
        *hessian += mass_matrix / (timestep * timestep);
    }
}