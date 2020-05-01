#include "LoboOptimizationSolver.h"
#include "LoboDynamic/LoboDynamic.h"

Lobo::LoboOptimizationSolver::LoboOptimizationSolver(DynamicModel *model_, int maxiter, double tol) : model(model_), maxiteration(maxiter), tolerance(tol)
{
    r = model->num_DOFs;
}

Lobo::LoboOptimizationSolver::~LoboOptimizationSolver() {}
