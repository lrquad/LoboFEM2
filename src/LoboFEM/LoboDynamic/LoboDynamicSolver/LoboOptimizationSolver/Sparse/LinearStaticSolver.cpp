#include "LinearStaticSolver.h"
#include "LoboDynamic/LoboDynamic.h"
#include <fstream>

Lobo::LinearStaticSolver::LinearStaticSolver(Lobo::DynamicModel *model_):Lobo::LoboOptimizationSolver(model_,0,0)
{
    q.resize(r);
    q.setZero();
}

void Lobo::LinearStaticSolver::precompute()
{
    std::cout<<"LinearStaticSolver::precompute"<<std::endl;
    model->getSparseTopoloty(hessian);
    jacobi.resize(model->num_DOFs);
    jacobi.setZero();
    
    int flags_all = 0; 
    flags_all |= Computeflags_second|Computeflags_reset;
    this->model->computeEnergySparse(&q,&energy,&jacobi,&hessian,flags_all);
    eigen_linearsolver.compute(hessian);
    std::cout<<"LinearStaticSolver::precompute end"<<std::endl;
}

void Lobo::LinearStaticSolver::solve(Eigen::VectorXd *initialGuessq)
{
    int flags_all = 0;
    flags_all |= Computeflags_fisrt|Computeflags_reset;
    q = *initialGuessq;
    this->model->computeEnergySparse(&q,&energy,&jacobi,&hessian,flags_all);
    jacobi*=-1.0;
    Eigen::VectorXd result_ = eigen_linearsolver.solve(jacobi);
    q = q+result_;
    
}

void Lobo::LinearStaticSolver::getResult(Eigen::VectorXd *result)
{
    *result = q;
}