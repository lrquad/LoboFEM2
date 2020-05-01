#pragma once
#include <string>
#include <vector>
#include <sstream>
#include "Utils/pugixml/pugixml.hpp"
#include <iostream>

namespace Lobo {

enum DynamicSceneFlag {
    dynamicsceneflag_stop = 1 << 0,
    dynamicsceneflag_play = 1 << 1  // generted mesh by tetgen
    };

class LoboScene;
class LoboTetMesh;
class LoboShader;

class DynamicSimulator;

std::string getPath(const char* filename);

class LoboDynamicScene {
   public:
    LoboDynamicScene(LoboScene* scene_);
    ~LoboDynamicScene();

    virtual void drawImGui(bool* p_open = NULL);
    virtual void runXMLscript(pugi::xml_node &scene_node);

    virtual void mouseRectSelect();
    
    virtual void update();  // update every frame
    virtual void stepForward();
    virtual void setPlay(){status_flags = dynamicsceneflag_play;};
    virtual void setStop(){status_flags = dynamicsceneflag_stop;};

    virtual int getStep();
    virtual int getSaveStep();

    virtual void paintGL(LoboShader* shader, bool depth_render = false);
    virtual void initialGL();
    virtual void updateGL();  // update vertex buffer
    virtual void deleteGL();

    virtual void bindTetMesh(int trimesh_id, const char* filebase,
                             bool loadfile, bool binary = true);

    std::vector<LoboTetMesh*> tetmesh_in_scene;
    std::vector<DynamicSimulator*> dynamic_solvers;
    
    std::vector<int> tetmesh_triId;
    LoboScene* scene;

protected:
    //system command check and mkdir folder we need
    void prepareDir();
    void drawImGuiTetMesh();
    void drawImGuiSolvers();

    int status_flags;
};

}  // namespace Lobo