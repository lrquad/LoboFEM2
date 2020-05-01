#include "NewtonLineSearch.h"
#include "LoboDynamic/LoboDynamic.h"
#include <fstream>

Lobo::NewtonLineSearch::NewtonLineSearch(DynamicModel *model_, int maxiter, double tol) : LoboOptimizationSolver(model_, maxiter, tol)
{
    q.resize(r);
    q.setZero();
}

Lobo::NewtonLineSearch::~NewtonLineSearch()
{

}

void Lobo::NewtonLineSearch::precompute()
{
    model->getSparseTopoloty(hessian);
    jacobi.resize(model->num_DOFs);
    jacobi.setZero();
}

void Lobo::NewtonLineSearch::solve(Eigen::VectorXd* initialGuessq)
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
        model->computeEnergySparse(&q,&energy,&jacobi,&hessian,flags_all);
        jacobi*=-1;

        if(std::abs(pre_energy-energy)/std::abs(energy+1e-15) < tolerance)
        {
            break;
        }else
        {
            pre_energy = energy;
        }
    
        #ifdef EIGEN_USE_MKL_ALL
        Eigen::PardisoLDLT<Eigen::SparseMatrix<double>> LDLT;
        LDLT.pardisoParameterArray()[33] = 1;
		LDLT.pardisoParameterArray()[59] = 0;
        #else // DEBUG
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> LDLT;
        #endif

        LDLT.compute(hessian);
        
        Eigen::VectorXd result_ = LDLT.solve(jacobi);

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
            model->computeEnergySparse(&q_k_1,&newton_e_k_1,&newton_g_k_1,NULL,flags_energy);
            bool wolf1 = newton_e_k_1 <= energy + c1*newton_stepping*(-jacobi.dot(newton_direction));
            bool wolf2 = true;
            wolfecondition = wolf1&&wolf2;
			if (!wolfecondition)
				newton_stepping *= 0.7;
        }
        q = q + newton_direction*newton_stepping;
    }
    //testGradient();
}

void Lobo::NewtonLineSearch::getResult(Eigen::VectorXd* result)
{
    *result = q;
}

void Lobo::NewtonLineSearch::testGradient()
{
    int flags_all = 0; 
    flags_all |= Computeflags_energy|Computeflags_fisrt|Computeflags_reset;

    double energy_ori;
    model->computeEnergySparse(&q,&energy_ori,&jacobi,&hessian,flags_all);
    double h = 1e-6;
    std::cout<<energy_ori <<" " << jacobi.data()[7*3]<<std::endl;
    q.data()[7*3]+=h;
    model->computeEnergySparse(&q,&energy,&jacobi,&hessian,flags_all);
    std::cout<<energy <<" " << jacobi.data()[7*3]<<std::endl;
    
    //check element indices
   
    std::cout<<(energy-energy_ori)/h<<std::endl;
}