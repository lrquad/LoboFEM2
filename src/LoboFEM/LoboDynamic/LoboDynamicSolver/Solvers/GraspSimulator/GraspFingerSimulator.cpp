#include "GraspFingerSimulator.h"
#include "LoboDynamic/LoboDynamic.h"
#include "OpenGLutils/LoboScene.h"
#include <Eigen/Geometry>

Lobo::GraspFingerSimulator::GraspFingerSimulator(
    Lobo::LoboDynamicScene *parent_scene_)
    : DynamicSimulator(parent_scene_) {
    contact_index = NULL;
    trimesh = NULL;
    simulation_step = 0;
}

Lobo::GraspFingerSimulator::~GraspFingerSimulator() {}

void Lobo::GraspFingerSimulator::drawImGui() {
    ImGui::Text("Finger mover");
    DynamicSimulator::drawImGui();
}

void Lobo::GraspFingerSimulator::paintGL(LoboShader *shader)
{

}

void Lobo::GraspFingerSimulator::runXMLscript(pugi::xml_node &solver_node) {
    DynamicSimulator::runXMLscript(solver_node);
    if (solver_node.child("TrimeshId")) {
        int triid = solver_node.child("TrimeshId").text().as_int();
        trimesh = this->parent_scene->scene->getMesh(triid);
    }
    if (solver_node.child("ContactPoints")) {
        contact_index = solver_node.child("ContactPoints").text().as_int();
        std::cout << "contact index" << contact_index << std::endl;
    }

    if (solver_node.attribute("precompute").as_bool()) {
        precompute();
    }
}

void Lobo::GraspFingerSimulator::precompute() {
    DynamicSimulator::precompute();
    std::cout << "Grasp simulator precomptued" << std::endl;

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

    // move to index
    Eigen::Vector3d target_position =
        this->bind_tetMesh->getNodeRestPosition(contact_index);

    // compute node normal direction
    contact_normal = this->bind_tetMesh->getNodeNormal(contact_index);
    target_position += contact_normal * 0.12;

    Eigen::Vector3d translation_ = target_position - tri_mesh_center;

    for (int i = 0; i < numtrinode; i++) {
        buffer.data()[i * 3 + 0] += translation_.data()[0];
        buffer.data()[i * 3 + 1] += translation_.data()[1];
        buffer.data()[i * 3 + 2] += translation_.data()[2];
    }

    trimesh->updateVertices(buffer.data());

    // force direction
    contact_normal = target_position;
    contact_normal.data()[1] = 0.0;
    contact_normal.normalize();
}

void Lobo::GraspFingerSimulator::stepForward() {
    int numtrinode = trimesh->attrib.vertices.size() / 3;
    Eigen::VectorXd buffer(numtrinode * 3);
    buffer.setZero();
    trimesh->getCurVertices(buffer.data());
    double speed = 0.001;
    if (simulation_step < 80) {
        for (int i = 0; i < numtrinode; i++) {
            buffer.data()[i * 3 + 0] += -contact_normal.data()[0] * speed;
            buffer.data()[i * 3 + 1] += -contact_normal.data()[1] * speed;
            buffer.data()[i * 3 + 2] += -contact_normal.data()[2] * speed;
        }
    } else if (simulation_step < 600) {
        for (int i = 0; i < numtrinode; i++) {
            buffer.data()[i * 3 + 1] += speed*0.6;
        }
    } else if (simulation_step < 1200) {
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(speed * 2.5, Eigen::Vector3d::UnitY());

        for (int i = 0; i < numtrinode; i++) {
            Eigen::Vector3d tmp;
            tmp.data()[0] = buffer.data()[i * 3 + 0];
            tmp.data()[1] = buffer.data()[i * 3 + 1];
            tmp.data()[2] = buffer.data()[i * 3 + 2];
            tmp = m * tmp;
            buffer.data()[i * 3 + 0] = tmp.data()[0];
            buffer.data()[i * 3 + 1] = tmp.data()[1];
            buffer.data()[i * 3 + 2] = tmp.data()[2];
        }

    } else if (simulation_step < 1800) {
        for (int i = 0; i < numtrinode; i++) {
            buffer.data()[i * 3 + 2] += speed*0.8;
        }
    } else {
        for (int i = 0; i < numtrinode; i++) {
            buffer.data()[i * 3 + 1] = 100;
        }
    }

    trimesh->updateVertices(buffer.data());

    simulation_step++;
}

int Lobo::GraspFingerSimulator::getCurStep() { return 0; }