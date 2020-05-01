#pragma once
#include "LoboDynamic/LoboDynamicSolver/LoboOptimizationSolver/LoboOptimizationSolver.h"

namespace Lobo
{
    class NewtonLineSearch:public LoboOptimizationSolver
    {
    private:
        /* data */
    public:
        NewtonLineSearch(DynamicModel* model_,int maxiter,double tol);
        ~NewtonLineSearch();

        virtual void precompute();
        virtual void solve(Eigen::VectorXd* initialGuessq);
        virtual void getResult(Eigen::VectorXd* result);

        Eigen::VectorXd q;
        Eigen::SparseMatrix<double> hessian;
        Eigen::VectorXd jacobi;
        double energy;

    protected:

        virtual void testGradient();

    };
}
