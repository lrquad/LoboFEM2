#pragma once
#include "LoboDynamic/LoboDynamicSolver/LoboDynamicModel/DynamicModel.h"
#include "Utils/pugixml/pugiconfig.hpp"
#include "LoboMesh/LoboMesh.h"
#include "LoboDynamic/LoboDynamicScene.h"

namespace Lobo {
class LoboTetMesh;

class GraspContactModel : public DynamicModel {
   public:
    GraspContactModel(LoboDynamicScene* scene_, LoboTetMesh* tetmesh_);
    ~GraspContactModel();

    virtual void precompute();

    virtual void paintGL(LoboShader *shader);

    virtual void runXMLscript(pugi::xml_node& xml_node);

    virtual void computeEnergySparse(Eigen::VectorXd* free_variables,
                                     double* energy, Eigen::VectorXd* jacobi,
                                     Eigen::SparseMatrix<double>* hessian,
                                     int computationflags);
    virtual void computeEnergyDense(Eigen::VectorXd* free_variables,
                                    double* energy, Eigen::VectorXd* jacobi,
                                    Eigen::MatrixXd* hessian,
                                    int computationflags){};

    virtual void setContactPointsIndex(std::vector<int> index_){};

    virtual void setpForward(int step);


    double weight_stiffness;
    double friction_ratio;
    double radius;
    Eigen::VectorXd q_1;
    Eigen::VectorXd q_vel_1;

   protected:
    std::vector<int> finger_list;
    std::vector<int> contact_points_list;  // the grasping force will apply to

    std::vector<Lobo::LoboMesh*> trimesh_list;
    std::vector<Eigen::Vector3d> contact_normal;
    std::vector<Eigen::Vector3d> contact_center;

    std::vector<Eigen::Vector3d> contact_pressure;
    std::vector<Eigen::Vector3d> contact_friction_force;
    Eigen::Vector3d net_contact_force;

    LoboTetMesh* tetmesh;
    LoboDynamicScene* scene;
};
}  // namespace Lobo