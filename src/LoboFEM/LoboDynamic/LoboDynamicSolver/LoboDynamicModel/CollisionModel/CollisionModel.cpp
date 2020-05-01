
#include "CollisionModel.h"
#include "LoboMesh/LoboMesh.h"
#include "Collision/CollisionDector/BVHCollisionDetector.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"

Lobo::CollisionModel::CollisionModel(LoboTetMesh *tetmesh_)
    : tetmesh(tetmesh_) {
    weight_stiffness = 100.0;  // default value;
    friction_weight_stiffness = 0.1;
    q_1.resize(num_DOFs);
    q_1.setZero();
}

Lobo::CollisionModel::~CollisionModel() {}

void Lobo::CollisionModel::runXMLscript(pugi::xml_node &xml_node) {
    Lobo::DynamicModel::runXMLscript(xml_node);

    if (xml_node.attribute("weight")) {
        weight_stiffness = xml_node.attribute("weight").as_double();
    }

    if (xml_node.attribute("friction")) {
        friction_weight_stiffness = xml_node.attribute("friction").as_double();
    }
}

void Lobo::CollisionModel::computeEnergySparse(
    Eigen::VectorXd *free_variables, double *energy, Eigen::VectorXd *jacobi,
    Eigen::SparseMatrix<double> *hessian, int computationflags) {
    if (computationflags & Computeflags_reset) {
        if (computationflags & Computeflags_energy) *energy = 0;

        if (computationflags & Computeflags_fisrt) jacobi->setZero();

        if (computationflags & Computeflags_second) {
            for (int i = 0; i < hessian->outerSize(); ++i)
                for (Eigen::SparseMatrix<double>::InnerIterator it(*hessian, i);
                     it; ++it) {
                    it.valueRef() = 0;
                }
        }
    }

    BVHCollisionDetector *detector = tetmesh->getBindMesh()->bvh_dectector;
    int numcollision = detector->getNumCollideInfoSize();
    // add constrain energy

    for (int i = 0; i < numcollision; i++) {
        CollideInfo info = detector->getCollideInfo(i);
        int tri_node = info.nodeid;
        int ele_node = tetmesh->tri_vertices_idl[tri_node];
        Eigen::Vector3d target_displacement = info.norms * info.displacement;

        for (int j = 0; j < 3; j++) {
            int dof_id = ele_node * 3 + j;
            double cur_dis =
                free_variables->data()[dof_id] - q_1.data()[dof_id];

            double loss = cur_dis*info.norms[j] + info.displacement;
            double friction_loss = cur_dis - cur_dis*info.norms[j];
            if (computationflags & Computeflags_energy) {

                *energy += weight_stiffness*loss*loss;
                //add friction
                *energy += friction_weight_stiffness*friction_loss*friction_loss;
            }

            if (computationflags & Computeflags_fisrt)
            {
                jacobi->data()[dof_id]+=2.0*weight_stiffness*loss*info.norms[j];
                 //add friction
                jacobi->data()[dof_id]+=2.0*friction_weight_stiffness*friction_loss*(1.0-info.norms[j]);
            }

            if (computationflags & Computeflags_second)
            {
                 hessian->valuePtr()[diagonal_[dof_id]]+=2.0*weight_stiffness*info.norms[j]*info.norms[j];
                  //add friction
                 hessian->valuePtr()[diagonal_[dof_id]]+=2.0*friction_weight_stiffness*(1.0-info.norms[j])*(1.0-info.norms[j]);
            }
        }
    }
}