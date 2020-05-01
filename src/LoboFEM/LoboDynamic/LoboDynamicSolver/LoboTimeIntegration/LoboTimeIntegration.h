#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Utils/pugixml/pugixml.hpp"

namespace Lobo {
class KineticModel;

enum IntegratorFlags_
{
    IntegratorFlags_None = 0,
    IntegratorFlags_recordq = 1 << 0,
    IntegratorFlags_recorderror = 1 << 1
};

class LoboTimeIntegration {
   private:
    /* data */
   public:
    LoboTimeIntegration(int num_DOfs_, double damping_ratio_, double timestep_,int skip_steps_, int flags_);
    ~LoboTimeIntegration();
    

    virtual void runXMLscript(pugi::xml_node &xml_node){};
    virtual void precompute() = 0;
    virtual void stepFoward() = 0;

    std::vector<Eigen::VectorXd> sequence_q;
	std::vector<Eigen::VectorXd> sequence_r;

    //free varibles
    Eigen::VectorXd q;
    Eigen::VectorXd q_vel;
    Eigen::VectorXd q_accel;

    Eigen::VectorXd q_residual;
    Eigen::VectorXd q_delta;

    Eigen::VectorXd q_1;
    Eigen::VectorXd q_vel_1;
    Eigen::VectorXd q_accel_1;


    //time integraion settings
    double damping_mass_coef;
	double damping_stiffness_coef;
	int r; // number of reduced DOFs;
	
    double timestep;
    int step;
	int skip_steps;
    double damping_ratio;

    int flags;

};

}  // namespace Lobo