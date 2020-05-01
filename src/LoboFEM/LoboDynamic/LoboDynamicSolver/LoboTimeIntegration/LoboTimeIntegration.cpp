#include "LoboTimeIntegration.h"

Lobo::LoboTimeIntegration::LoboTimeIntegration(int num_DOfs_,
                                               double damping_ratio_,
                                               double timestep_,
                                               int skip_steps_, int flags_)
    : r(num_DOfs_),
      damping_ratio(damping_ratio_),
      timestep(timestep_),
      skip_steps(skip_steps_),flags(flags_){
    
    q.resize(r);
    q_vel.resize(r);
    q_accel.resize(r);

    q_residual.resize(r);
    q_delta.resize(r);

    q_1.resize(r);
    q_vel_1.resize(r);
    q_accel_1.resize(r);

    q.setZero();
    q_vel.setZero();
    q_accel.setZero();
    q_residual.setZero();
    q_delta.setZero();
    q_1.setZero();
    q_vel_1.setZero();
    q_accel_1.setZero();

    step = 0;

}

Lobo::LoboTimeIntegration::~LoboTimeIntegration() {}
