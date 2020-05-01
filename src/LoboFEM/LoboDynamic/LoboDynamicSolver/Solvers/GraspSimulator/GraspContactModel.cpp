#include "GraspContactModel.h"
#include <fstream>
#include "LoboDynamic/LoboDynamic.h"
#include "OpenGLutils/LoboScene.h"
#include "Shaders/LoboShader.h"

Lobo::GraspContactModel::GraspContactModel(Lobo::LoboDynamicScene *scene_,
                                           Lobo::LoboTetMesh *tetmesh_)
    : tetmesh(tetmesh_) {
    weight_stiffness = 1000.0;
    friction_ratio = 1.0;
    radius = 0.1;
    this->scene = scene_;
    net_contact_force.setZero();
}

Lobo::GraspContactModel::~GraspContactModel() {}

void Lobo::GraspContactModel::runXMLscript(pugi::xml_node &xml_node) {
    trimesh_list.clear();
    contact_points_list.clear();
    finger_list.clear();

    for (pugi::xml_node index_node : xml_node.children("Index")) {
        int contact_point_index = index_node.text().as_int();
        contact_points_list.push_back(contact_point_index);
        finger_list.push_back(contact_point_index);
        if (index_node.attribute("triid")) {
            int tri_id = index_node.attribute("triid").as_int();
            trimesh_list.push_back(scene->scene->getMesh(tri_id));
        }
    }

    if (xml_node.attribute("weight")) {
        weight_stiffness = xml_node.attribute("weight").as_double();
    }

    if (xml_node.attribute("friction")) {
        friction_ratio = xml_node.attribute("friction").as_double();
    }
    precompute();
}

void Lobo::GraspContactModel::precompute() {
    contact_normal.clear();
    contact_center.clear();
    contact_pressure.clear();
    contact_friction_force.clear();

    // add the neighbor points
    int num_sur_nodes = tetmesh->surface_vertice_map_inverse.size();
    std::vector<int> neighbor_points;
    for (int i = 0; i < finger_list.size(); i++) {
        Eigen::Vector3d target_position =
            this->tetmesh->getNodeRestPosition(finger_list[i]);
        for (int j = 0; j < num_sur_nodes; j++) {
            int nodeid = tetmesh->surface_vertice_map_inverse[j];
            Eigen::Vector3d node_ori_p = tetmesh->getNodeRestPosition(nodeid);
            double distance = (node_ori_p - target_position).norm();
            if (distance < 0.01 && nodeid != finger_list[i]) {
                neighbor_points.push_back(nodeid);
            }
        }
    }

    for (int i = 0; i < neighbor_points.size(); i++) {
        contact_points_list.push_back(neighbor_points[i]);
    }

    // update visual
    for (int n = 0; n < finger_list.size(); n++) {
        Lobo::LoboMesh *trimesh = trimesh_list[n];
        int numtrinode = trimesh->attrib.vertices.size() / 3;
        Eigen::VectorXd buffer(numtrinode * 3);
        trimesh->getCurVertices(buffer.data());
        Eigen::Vector3d tri_mesh_center;
        tri_mesh_center.setZero();
        for (int i = 0; i < numtrinode; i++) {
            tri_mesh_center.data()[0] += buffer.data()[i * 3 + 0];
            tri_mesh_center.data()[1] += buffer.data()[i * 3 + 1];
            tri_mesh_center.data()[2] += buffer.data()[i * 3 + 2];
        }
        tri_mesh_center /= numtrinode;
        Eigen::Vector3d target_position =
            this->tetmesh->getNodeRestPosition(finger_list[n]);
        Eigen::Vector3d contact_normal_ =
            this->tetmesh->getNodeNormal(finger_list[n]);
        target_position += contact_normal_ * 0.0;
        Eigen::Vector3d translation_ = target_position - tri_mesh_center;
        for (int i = 0; i < numtrinode; i++) {
            buffer.data()[i * 3 + 0] += translation_.data()[0];
            buffer.data()[i * 3 + 1] += translation_.data()[1];
            buffer.data()[i * 3 + 2] += translation_.data()[2];
        }
        trimesh->updateVertices(buffer.data());
    }

    for (int n = 0; n < contact_points_list.size(); n++) {
        // move to index
        Eigen::Vector3d target_position =
            this->tetmesh->getNodeRestPosition(contact_points_list[n]);
        Eigen::Vector3d contact_normal_ =
            this->tetmesh->getNodeNormal(contact_points_list[n]);
        target_position += contact_normal_ * 0.0;
        contact_normal.push_back(contact_normal_);
        contact_center.push_back(target_position);
        contact_pressure.push_back(Eigen::Vector3d::Zero());
        contact_friction_force.push_back(Eigen::Vector3d::Zero());
    }
}

void Lobo::GraspContactModel::paintGL(LoboShader *shader) {
    glLineWidth(2.5);
    glColor3f(1.0, 0.0, 0.0);
    double force_scale = 100.0;
    double friction_coef = 0.6;
    for (int i = 0; i < contact_points_list.size(); i++) {
        int nodeid = contact_points_list[i];
        Eigen::Vector3d node_n = tetmesh->getNodeNormal(nodeid);
        Eigen::Vector3d node_p = tetmesh->getNodeCurPosition(nodeid);
        Eigen::Vector3d arrow_end;
        node_p += node_n * 0.2;
        glColor3f(0.0, 0.0, 1.0);
        glBegin(GL_LINES);
        glVertex3d(node_p.data()[0], node_p.data()[1], node_p.data()[2]);
        // node_p.data()[1] += 0.1;
        arrow_end = node_p + contact_pressure[i] * force_scale;

        glVertex3f(arrow_end.data()[0], arrow_end.data()[1],
                   arrow_end.data()[2]);
        glEnd();

        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        glVertex3d(node_p.data()[0], node_p.data()[1], node_p.data()[2]);
        // node_p.data()[1] += 0.1;
        arrow_end = node_p + contact_friction_force[i] * force_scale;

        glVertex3f(arrow_end.data()[0], arrow_end.data()[1],
                   arrow_end.data()[2]);
        glEnd();

        // check the condition
        if (contact_friction_force[i].norm() >
            contact_pressure[i].norm() * friction_coef) {
            glColor3f(1.0, 0.0, 0.0);
            node_n = tetmesh->getNodeNormal(nodeid);
            glBegin(GL_LINES);
            glVertex3d(node_p.data()[0], node_p.data()[1], node_p.data()[2]);
            glVertex3d(node_p.data()[0], node_p.data()[1] + 1.0,
                       node_p.data()[2]);
            glEnd();
        }
    }

    net_contact_force *= force_scale;
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0);
    glVertex3d(net_contact_force.data()[0], net_contact_force.data()[1],
               net_contact_force.data()[2]);
    glEnd();
}

void Lobo::GraspContactModel::computeEnergySparse(

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

    // collision detect
    net_contact_force.setZero();
    for (int i = 0; i < contact_points_list.size(); i++) {
        int nodeid = contact_points_list[i];
        Eigen::Vector3d node_position = tetmesh->getNodeRestPosition(nodeid);
        node_position.data()[0] += free_variables->data()[nodeid * 3 + 0];
        node_position.data()[1] += free_variables->data()[nodeid * 3 + 1];
        node_position.data()[2] += free_variables->data()[nodeid * 3 + 2];
        Eigen::Vector3d constrain_force;
        constrain_force.setZero();
        for (int j = 0; j < 3; j++) {
            int dof_id = nodeid * 3 + j;
            double loss = node_position.data()[j] - contact_center[i].data()[j];

            constrain_force.data()[j] = -2.0 * loss;

            if (computationflags & Computeflags_energy) {
                *energy += weight_stiffness * loss * loss;
            }

            if (computationflags & Computeflags_fisrt) {
                jacobi->data()[dof_id] += 2.0 * weight_stiffness * loss;
            }

            if (computationflags & Computeflags_second) {
                hessian->valuePtr()[diagonal_[dof_id]] +=
                    2.0 * weight_stiffness;
            }
        }
        net_contact_force += constrain_force;

        Eigen::Vector3d node_n = tetmesh->getNodeNormal(nodeid);
        double projected_value = constrain_force.dot(node_n);
        contact_pressure[i] = projected_value * node_n;
        contact_friction_force[i] = constrain_force - contact_pressure[i];
    }
}

void Lobo::GraspContactModel::setpForward(int step) {
    double speed = 0.002;
    int steps[4];
    steps[0] = 50;
    steps[1] = 100;
    steps[2] = 400;
    steps[3] = 700;
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(speed * 2.5, Eigen::Vector3d::UnitY());

    for (int i = 0; i < contact_points_list.size(); i++) {
        if (step < steps[0]) {
            contact_center[i].data()[0] += -contact_normal[i].data()[0] * speed;
            contact_center[i].data()[1] += -contact_normal[i].data()[1] * speed;
            contact_center[i].data()[2] += -contact_normal[i].data()[2] * speed;
        } else if (step < steps[1]) {
            contact_center[i].data()[1] += speed * 3.0;
        } else if (step < steps[2]) {
            contact_center[i] = m * contact_center[i];
        } else if (step < steps[3]) {
            contact_center[i].data()[2] += speed * 0.8;
        } else {
            this->trigger = false;
            contact_pressure[i].setZero();
            contact_friction_force[i].setZero();
        }
    }

    for (int i = 0; i < finger_list.size(); i++) {
        Lobo::LoboMesh *trimesh = trimesh_list[i];
        int numtrinode = trimesh->attrib.vertices.size() / 3;
        Eigen::VectorXd buffer(numtrinode * 3);
        buffer.setZero();
        trimesh->getCurVertices(buffer.data());

        if (step < steps[0]) {
            for (int j = 0; j < numtrinode; j++) {
                buffer.data()[j * 3 + 0] +=
                    -contact_normal[i].data()[0] * speed;
                buffer.data()[j * 3 + 1] +=
                    -contact_normal[i].data()[1] * speed;
                buffer.data()[j * 3 + 2] +=
                    -contact_normal[i].data()[2] * speed;
            }
        } else if (step < steps[1]) {
            for (int j = 0; j < numtrinode; j++) {
                buffer.data()[j * 3 + 1] += speed * 3.0;
            }
        } else if (step < steps[2]) {
            for (int j = 0; j < numtrinode; j++) {
                Eigen::Vector3d tmp;
                tmp.data()[0] = buffer.data()[j * 3 + 0];
                tmp.data()[1] = buffer.data()[j * 3 + 1];
                tmp.data()[2] = buffer.data()[j * 3 + 2];
                tmp = m * tmp;
                buffer.data()[j * 3 + 0] = tmp.data()[0];
                buffer.data()[j * 3 + 1] = tmp.data()[1];
                buffer.data()[j * 3 + 2] = tmp.data()[2];
            }

        } else if (step < steps[3]) {
            for (int j = 0; j < numtrinode; j++) {
                buffer.data()[j * 3 + 2] += speed * 0.8;
            }
        } else {
            for (int j = 0; j < numtrinode; j++) {
                buffer.data()[j * 3 + 1] = 100;
            }
        }

        trimesh->updateVertices(buffer.data());
    }
}
