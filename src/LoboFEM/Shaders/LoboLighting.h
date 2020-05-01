#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>
#include "imgui.h"

namespace Lobo {
static float vertices[] = {
    -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,
    -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,
    -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,  0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,
    -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, -0.5f, -0.5f,
    -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,
    0.5f,  0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f,
    0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,  -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f,
    -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f,
    -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,
    0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f,
};
class LoboShader;
class LoboLighting;

class LoboLightManager {
   public:
    LoboLightManager();
    ~LoboLightManager();

    virtual void drawImGui(bool* p_open = NULL);
    virtual void paintGL(LoboShader* shader);
    virtual void setLight(LoboShader* render_shader);
    virtual void setLightShadow(LoboShader* depth_shader, int lightid = 0);

    unsigned int getDepthFBO(int lightid);
    unsigned int getDepthMap(int lightid);
    bool getLightTrigger(int lightid);
    bool getLightCastShadow(int lightid);

    int getLightNum();
    void getTextureSize(unsigned int &w,unsigned int &h,int lightid);

    protected: 
    std::vector<LoboLighting*> lighting_list;
};

class LoboLighting {
   public:
    LoboLighting();
    ~LoboLighting(){};

    virtual void drawImGui();
    virtual void drawPointLightImGui();
    virtual void drawDirectionalLightImGui();
    virtual void paintGL(LoboShader* shader);
    virtual void setLight(LoboShader* render_shader, int lightid);
    virtual void setPointLight(LoboShader* render_shader,
                               std::string lightname);
    virtual void setDirectionalLight(LoboShader* render_shader,
                                     std::string lightname);

    virtual void setLightShadow(LoboShader* depth_shader,int lightid);
    virtual void setLightDepthShadow(LoboShader* depth_shader,int lightid);

    virtual void initShadowMap();

    glm::vec3 lightPos;
    glm::vec3 lightColor;
    glm::vec3 direction;

    float constant;
    float linear;
    float quadratic;
    bool trigger;
    bool cast_shadow;
    int light_type;

    unsigned int lightVAO;
    unsigned int depthMapFBO;
    unsigned int depthMap;
     unsigned int SHADOW_WIDTH ;
     unsigned int SHADOW_HEIGHT ;
};

}  // namespace Lobo