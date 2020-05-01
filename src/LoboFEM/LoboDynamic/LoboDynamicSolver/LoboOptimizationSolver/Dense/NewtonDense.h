#pragma once
#include "LoboDynamic/LoboDynamicSolver/LoboOptimizationSolver/LoboOptimizationSolver.h"

namespace Lobo
{
    class NewtonDense:public LoboOptimizationSolver
    {
    private:
        /* data */
    public:
        NewtonDense(DynamicModel* model_,int maxiter,double tol);
        ~NewtonDense();

        virtual void precompute();
        virtual void solve(Eigen::VectorXd* initialGuessq);
        virtual void getResult(Eigen::VectorXd* result);

        Eigen::VectorXd q;
        Eigen::MatrixXd hessian;
        Eigen::VectorXd jacobi;
        double energy;
    protected:
    };
}
