#pragma once
#include "LoboDynamic/LoboDynamicSolver/LoboTimeIntegration/LoboTimeIntegration.h"

namespace Lobo
{
    class KineticModel;
    class LoboOptimizationSolver;

    class ImplicitDenseIntegration: public LoboTimeIntegration
    {
    private:
        /* data */
    public:
        ImplicitDenseIntegration(KineticModel *kineticmodel_, int num_DOfs_, double damping_ratio_, double timestep_,int skip_steps_, int flags_);
        ~ImplicitDenseIntegration();

        virtual void runXMLscript(pugi::xml_node &xml_node);
        virtual void precompute();
        virtual void stepFoward();

    protected:
        KineticModel *kineticmodel;
        LoboOptimizationSolver* opsovler;
    };
}