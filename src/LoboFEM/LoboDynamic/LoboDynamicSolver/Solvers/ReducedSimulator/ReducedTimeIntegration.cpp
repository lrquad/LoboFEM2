#include "ReducedTimeIntegration.h"
#include "ReducedKineticModel.h"

Lobo::ReducedTimeIntegration::ReducedTimeIntegration(ReducedKineticModel* kineticmodel_,int full_DoFs_, double damping_ratio_, double timestep_, int skip_steps_, int flags_):ImplicitDenseIntegration(kineticmodel_, full_DoFs_, damping_ratio_, timestep_, skip_steps_, flags_)
{
    reduced_model = kineticmodel_;

    reduced_DOFs = reduced_model->phi.cols();


    q_reduced.resize(reduced_DOFs);
    q_reduced.setZero();
}

Lobo::ReducedTimeIntegration::~ReducedTimeIntegration()
{

}

void Lobo::ReducedTimeIntegration::precompute()
{
    Lobo::ImplicitDenseIntegration::precompute();
}

void Lobo::ReducedTimeIntegration::setInitLatentsq(Eigen::VectorXd &q_reduced_)
{
    q_reduced = q_reduced_;
    q = reduced_model->phi*q_reduced;
    q_1 = q;
}

void Lobo::ReducedTimeIntegration::stepFoward()
{
    q_1 = q;
    reduced_model->setKineticStatus(q_vel, q_1);
    opsovler->solve(&q_reduced);
    opsovler->getResult(&q_reduced);
    q = reduced_model->phi*q_reduced;
    q_vel_1 = q_vel;
    q_vel = (q - q_1) / timestep;
    q_vel *= damping_ratio;
    step++;
}