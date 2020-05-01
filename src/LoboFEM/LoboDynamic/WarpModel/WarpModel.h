#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

namespace Lobo
{
    class LoboTetMesh;

    class WarpModel
    {
        public:
        WarpModel(LoboTetMesh* tetmesh_,std::vector<int> &warp_node_list_);
        ~WarpModel();

        virtual void precompute() = 0;
        virtual void warp(Eigen::VectorXd &fullq) = 0;
        virtual void warpForce(Eigen::VectorXd &force,Eigen::VectorXd &q, bool T = true) = 0;

        protected:
        LoboTetMesh* tetmesh;
        std::vector<int> warp_node_list;

    };
}