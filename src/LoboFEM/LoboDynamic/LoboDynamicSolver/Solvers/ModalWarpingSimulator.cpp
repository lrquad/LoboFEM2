#include "ModalWarpingSimulator.h"
#include "LoboDynamic/LoboDynamic.h"
#include "LoboDynamic/WarpModel/ModalWarpingModel.h"
#include "imgui.h"

Lobo::ModalWarpingSimulator::ModalWarpingSimulator(Lobo::LoboDynamicScene *parent_scene):FullspaceSimulator(parent_scene)
{
    modal_warping_model = NULL;
}

Lobo::ModalWarpingSimulator::~ModalWarpingSimulator()
{
    delete modal_warping_model;
}

void Lobo::ModalWarpingSimulator::precompute()
{
    Lobo::FullspaceSimulator::precompute();
    //precompute modal warping matrices 
    std::vector<int> node_list;
    node_list.resize(bind_tetMesh->getNumVertex());
    for(int i=0;i<bind_tetMesh->getNumVertex();i++)
    {
        node_list[i] = i;
    }
    modal_warping_model = new ModalWarpingModel(bind_tetMesh,node_list);
    modal_warping_model->precompute();

}
void Lobo::ModalWarpingSimulator::stepForward()
{
    
    //kinetic_model->external_forces = kinetic_model->gravity_force*scale;
    kinetic_model->external_forces = kinetic_model->gravity_force;
    kinetic_model->external_forces.setZero();
    kinetic_model->external_forces+=bind_tetMesh->tet_vertice_force*0.01;
    modal_warping_model->warpForce(kinetic_model->external_forces,time_integraion->q,true);
    
    time_integraion->stepFoward();

    Eigen::VectorXd warped_q = time_integraion->q;
    modal_warping_model->warp(warped_q);
    bind_tetMesh->updateTetVertices(&(warped_q));
}