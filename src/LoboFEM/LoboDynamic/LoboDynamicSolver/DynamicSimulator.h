#pragma once

#include <vector>
#include <cstring>
#include "Utils/pugixml/pugixml.hpp"

namespace Lobo
{
class DynamicModel;
class LoboDynamicScene;
class LoboTetMesh;

class ConstrainModel;
class CollisionModel;
class LoboShader;

class DynamicSimulator
{
public:
    DynamicSimulator(LoboDynamicScene *parent_scene);
    ~DynamicSimulator();

    virtual void drawImGui();

    virtual void stepForward() = 0;
    virtual int getCurStep() = 0;

    virtual int getSaveCurStep(){return 0;};

    virtual void runXMLscript(pugi::xml_node &solver_node);

    virtual void precompute();

    virtual void setBindTetmeshFromScene(int tetmesh_id);

    virtual void paintGL(LoboShader *shader){};

    std::vector<DynamicModel *> models;

    //general models

    LoboDynamicScene *parent_scene;
    int target_tetmesh_id;
    LoboTetMesh *bind_tetMesh;
    int skipsteps;
};
} // namespace Lobo
