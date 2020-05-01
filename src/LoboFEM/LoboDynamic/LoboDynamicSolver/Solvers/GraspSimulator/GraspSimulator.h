#pragma once
#include "LoboDynamic/LoboDynamicSolver/DynamicSimulator.h"


namespace Lobo
{
class HyperelasticModel;
class KineticModel;
class LoboTimeIntegration;
class LoboTetMesh;
class GraspContactModel;
class LoboShader;
class GraspSimulator: public DynamicSimulator
{
public: 
    GraspSimulator(Lobo::LoboDynamicScene *parent_scene);
    ~GraspSimulator();

    virtual void drawImGui();

    virtual void runXMLscript(pugi::xml_node &solver_node);

    virtual void precompute();
    virtual void stepForward();
    virtual int getCurStep();
    virtual int getSaveCurStep();

    virtual void paintGL(LoboShader *shader);

    GraspContactModel* graspmodel;
    ConstrainModel *constrainmodel;
    CollisionModel *collisionmodel;
    HyperelasticModel *hyperelastic_model;
    KineticModel *kinetic_model;
    LoboTimeIntegration *time_integraion;

    std::vector<int> contact_points_list; // simulate the robot fingers

};


}; // namespace Lobo