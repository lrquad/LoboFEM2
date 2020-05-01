#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/PardisoSupport>

namespace Lobo
{
    class DynamicModel;

    class LoboOptimizationSolver
    {
    private:
        /* data */
    public:
        LoboOptimizationSolver(DynamicModel* model_,int maxiter,double tol);
        ~LoboOptimizationSolver();
        
        virtual void precompute() = 0;
        virtual void solve(Eigen::VectorXd* initialGuessq) = 0;
        virtual void getResult(Eigen::VectorXd* result) = 0;

    protected:

        DynamicModel* model;
        int maxiteration;
        double tolerance;
        int r;
    };

}