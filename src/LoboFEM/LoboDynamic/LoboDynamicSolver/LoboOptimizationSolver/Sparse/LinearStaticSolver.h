#pragma once

#include "LoboDynamic/LoboDynamicSolver/LoboOptimizationSolver/LoboOptimizationSolver.h"

namespace Lobo
{
class LinearStaticSolver : public LoboOptimizationSolver
{
public:
    LinearStaticSolver(DynamicModel *model_);
    
    virtual void precompute();
    virtual void solve(Eigen::VectorXd *initialGuessq);
    virtual void getResult(Eigen::VectorXd *result);

    Eigen::VectorXd q;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd jacobi;
    double energy;

#ifdef EIGEN_USE_MKL_ALL
    Eigen::PardisoLDLT<Eigen::SparseMatrix<double>> eigen_linearsolver;
#else // DEBUG
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> eigen_linearsolver;
#endif

};

} // namespace Lobo