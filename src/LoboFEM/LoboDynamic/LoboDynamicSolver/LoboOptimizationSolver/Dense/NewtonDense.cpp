#include "NewtonDense.h"
#include "LoboDynamic/LoboDynamic.h"
#include <fstream>
#include <iostream>

Lobo::NewtonDense::NewtonDense(DynamicModel *model_, int maxiter, double tol) : LoboOptimizationSolver(model_, maxiter, tol)
{
    q.resize(r);
    q.setZero();
}

Lobo::NewtonDense::~NewtonDense()
{

}   

void Lobo::NewtonDense::precompute()
{
    hessian.resize(r,r);
    jacobi.resize(r);
}

void Lobo::NewtonDense::solve(Eigen::VectorXd* initialGuessq)
{
    q = *initialGuessq;

    double newton_stepping = 1.0; //line length
    int flags_all = 0; 
    flags_all |= Computeflags_energy|Computeflags_fisrt|Computeflags_second|Computeflags_reset;
    int flags_energy = 0;
    flags_energy |= Computeflags_energy|Computeflags_reset;

    double pre_energy = DBL_MAX;

    for(int i=0;i<maxiteration;i++)
    {
        model->computeEnergyDense(&q,&energy,&jacobi,&hessian,flags_all);
        jacobi*=-1;

        double tmp = std::abs(pre_energy-energy)/std::abs(energy+1e-15);
        //std::cout<<tmp<<std::endl;
        if(tmp < tolerance)
        {
            break;
        }else
        {
            pre_energy = energy;
        }
    
        Eigen::VectorXd result_ = hessian.ldlt().solve(jacobi);

        //wolfe condition
        newton_stepping = 1.0;
		bool wolfecondition = false;
		double c1 = 0.0;
		double c2 = 0.9;
        //prepare wolf condition buffer
        Eigen::VectorXd newton_g_k_1(r);
        double newton_e_k_1 = 0.0;
        Eigen::VectorXd newton_direction = result_;

        while(!wolfecondition)
        {
            if(newton_stepping <0.01)
            {
                break;
            }
            Eigen::VectorXd q_k_1 = q + newton_direction*newton_stepping;
            model->computeEnergyDense(&q_k_1,&newton_e_k_1,&newton_g_k_1,NULL,flags_energy);
            bool wolf1 = newton_e_k_1 <= energy + c1*newton_stepping*(-jacobi.dot(newton_direction));
            bool wolf2 = true;
            wolfecondition = wolf1&&wolf2;
			if (!wolfecondition)
				newton_stepping *= 0.7;
        }
        q = q + newton_direction*newton_stepping;
    }
    
}

void Lobo::NewtonDense::getResult(Eigen::VectorXd* result)
{
    *result = q;
}