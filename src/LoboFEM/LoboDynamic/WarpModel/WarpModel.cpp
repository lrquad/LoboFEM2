#include "WarpModel.h"

Lobo::WarpModel::WarpModel(Lobo::LoboTetMesh* tetmesh_,std::vector<int> &warp_node_list_)
{
    this->tetmesh = tetmesh_;
    this->warp_node_list = warp_node_list_;
}

Lobo::WarpModel::~WarpModel()
{
    
}