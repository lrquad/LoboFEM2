#pragma once
#include "LoboMesh/LoboMesh.h"
#include "OpenGLutils/LoboScene.h"
#include "LoboDynamic/LoboDynamicScene.h"
#include "LoboVolumtricMesh/LoboTetMesh.h"
#include "OpenGLutils/LoboCamera.h"
#include "Shaders/LoboLighting.h"
#include "Shaders/LoboShader.h"
#include "LoboImGui/imfilebrowser.h"
#include "Utils/pugixml/pugixml.hpp"

namespace Lobo
{
class LoboDynamicScene;

static unsigned int SHADOW_WIDTH = 8192, SHADOW_HEIGHT = 8192;

class LoboFEM
{
public:
    LoboFEM();
    ~LoboFEM();

    virtual void initScreenBuffer();

    virtual void windowLoop(GLFWwindow *window);
    virtual void drawImGui();
    virtual void paintGL(LoboShader *shader);
    virtual void deleteGL();
    virtual void makeContext();
    virtual void setCurrentContext();
    void loadXMLfile(const char *filename);
    void framebuffer_size_callback(GLFWwindow* window, int width, int height);

    bool enablevsync;


protected:


    void showMainWindow(ImGui::FileBrowser *fileDialog, bool *p_open = NULL);
    void saveCurScreenImagePNG(const char* imagename);
    void saveCurScreenImagePNGAnimaition(const char* imagebase);


    ImGui::FileBrowser fileDialog;
    LoboScene *scene;
    LoboDynamicScene *dynamic_scene;

    LoboShader default_shader;
    LoboShader lighting_shader;
    LoboShader simpleDepthShader;
    LoboShader debugDepthQuad;
    LoboShader screenShader;
 
    LoboLightManager light_manager;
    Camera camera;
    ImVec4 clear_color;

    std::string config_file_path;
    pugi::xml_document xml_doc;

    unsigned int quadVAO;
    unsigned int quadVBO;
    unsigned int framebuffer;
    unsigned int textureColorbuffer;
    unsigned int textureColorBufferMultiSampled;
    unsigned int intermediateFBO;
    unsigned int screenTexture;
    unsigned int rbo;
    
    bool use_screen_buffer;
    bool export_screen_buffer;

    int multisamples;
};

} // namespace Lobo
