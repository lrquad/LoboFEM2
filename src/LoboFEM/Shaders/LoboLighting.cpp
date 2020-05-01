#include "LoboLighting.h"
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>
#include <sstream>
#include "Functions/deleteSTDvector.h"
#include "LoboShader.h"
#include "OpenGLutils/LoboCamera.h"

Lobo::LoboLightManager::LoboLightManager() {
    // 3 direction 3 point
    lighting_list.resize(6);
    for (int i = 0; i < lighting_list.size(); i++) {
        lighting_list[i] = new LoboLighting();
        lighting_list[i]->initShadowMap();
    }

    // initial setting
    for (int i = 0; i < 3; i++) {
        lighting_list[i]->trigger = true;
        lighting_list[i]->cast_shadow = true;
    }
    lighting_list[0]->lightColor = glm::vec3(0.98039, 0.83922, 0.64706);
    lighting_list[0]->lightPos = glm::vec3(6.0, 6.0, 6.0);
    lighting_list[1]->lightPos = glm::vec3(-6, 6.0, 6.0);
    lighting_list[2]->lightPos = glm::vec3(0.0, 6.0, -6.0);
}

Lobo::LoboLightManager::~LoboLightManager() {
    deleteStdvectorPointer(lighting_list);
}

void Lobo::LoboLightManager::drawImGui(bool *p_open) {
    ImGui::Begin("Lights", NULL);
    for (int i = 0; i < lighting_list.size(); i++) {
        ImGui::PushID(i);
        lighting_list[i]->drawImGui();
        ImGui::PopID();
    }
    ImGui::End();
}

void Lobo::LoboLightManager::paintGL(LoboShader *shader) {
    for (int i = 0; i < lighting_list.size(); i++) {
        if (lighting_list[i]->trigger) lighting_list[i]->paintGL(shader);
    }
}

void Lobo::LoboLightManager::setLight(LoboShader *render_shader) {
    for (int i = 0; i < lighting_list.size(); i++) {
        lighting_list[i]->setLight(render_shader, i);

        if (lighting_list[i]->trigger) {
            lighting_list[i]->setLightShadow(render_shader, i);
        }
    }
}

void Lobo::LoboLightManager::setLightShadow(LoboShader *depth_shader,
                                            int lightid) {
    lighting_list[lightid]->setLightDepthShadow(depth_shader, lightid);
}

unsigned int Lobo::LoboLightManager::getDepthFBO(int lightid) {
    return lighting_list[lightid]->depthMapFBO;
}
unsigned int Lobo::LoboLightManager::getDepthMap(int lightid) {
    return lighting_list[lightid]->depthMap;
}

bool Lobo::LoboLightManager::getLightTrigger(int lightid) {
    return lighting_list[lightid]->trigger;
}
bool Lobo::LoboLightManager::getLightCastShadow(int lightid) {
    return lighting_list[lightid]->cast_shadow;
}

int Lobo::LoboLightManager::getLightNum() { return lighting_list.size(); }

void Lobo::LoboLightManager::getTextureSize(unsigned int &w, unsigned int &h,
                                            int lightid) {
    h = lighting_list[lightid]->SHADOW_HEIGHT;
    w = lighting_list[lightid]->SHADOW_WIDTH;
}

Lobo::LoboLighting::LoboLighting() {
    lightPos = glm::vec3(0.0f, 1.0f, 1.0f);
    lightColor = glm::vec3(0.7f, 0.7f, 0.7f);
    direction = glm::vec3(0.0f, -1.0f, -1.0f);
    constant = 1.0;
    linear = 0.09;
    quadratic = 0.032;
    light_type = 0;
    SHADOW_WIDTH = 4096;
    SHADOW_HEIGHT = 4096;
    trigger = false;
    cast_shadow = false;

    glGenVertexArrays(1, &lightVAO);
    glBindVertexArray(lightVAO);

    unsigned int VBO;
    glGenBuffers(1, &VBO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    // we only need to bind to the VBO (to link it with glVertexAttribPointer),
    // no need to fill it; the VBO's data already contains all we need (it's
    // already bound, but we do it again for educational purposes)

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Lobo::LoboLighting::drawImGui() {
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    ImGui::Checkbox("On/Off", &trigger);
    ImGui::SameLine();
    const char *items[] = {"Point light", "Directional light"};
    ImGui::Combo("combo", &light_type, items, IM_ARRAYSIZE(items));
    if (trigger) {
        ImGui::Checkbox("Shadow", &cast_shadow);

        ImGui::DragFloat("lightPos.x", &lightPos.x, 0.05f);
        ImGui::DragFloat("lightPos.y", &lightPos.y, 0.05f);
        ImGui::DragFloat("lightPos.z", &lightPos.z, 0.05f);
        ImGui::ColorEdit3("light color", &lightColor.r);
        if (light_type == 0) drawPointLightImGui();
        if (light_type == 1) drawDirectionalLightImGui();
    }
}

void Lobo::LoboLighting::paintGL(LoboShader *shader) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, lightPos);
    model = glm::scale(model, glm::vec3(0.2f));  // a smaller cube
    shader->setMat4("model", model);
    shader->setVec3("lightColor", lightColor);

    glBindVertexArray(lightVAO);

    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Lobo::LoboLighting::drawPointLightImGui() {
    if (ImGui::TreeNode("PointLight##2")) {
        ImGui::DragFloat("constant", &constant, 0.1f);
        ImGui::DragFloat("linear", &linear, 0.01f);
        ImGui::DragFloat("quadratic", &quadratic, 0.001f);
        ImGui::TreePop();
        ImGui::Separator();
    }
}

void Lobo::LoboLighting::drawDirectionalLightImGui() {
    if (ImGui::TreeNode("DirectionalLight##2")) {
        ImGui::DragFloat("direction.x", &direction.x, 0.05f);
        ImGui::DragFloat("direction.y", &direction.y, 0.05f);
        ImGui::DragFloat("direction.z", &direction.z, 0.05f);
        ImGui::TreePop();
        ImGui::Separator();
    }
}

void Lobo::LoboLighting::setLight(LoboShader *render_shader, int lightid) {
    std::ostringstream stringStream;
    stringStream << "lights[" << lightid << "]";
    std::string lightname = stringStream.str();

    if (light_type == 0) {
        setPointLight(render_shader, lightname);
    }
    if (light_type == 1) {
        setDirectionalLight(render_shader, lightname);
    }

    render_shader->setInt(lightname + ".light_type", light_type);
    render_shader->setBool(lightname + ".trigger", trigger);
    render_shader->setBool(lightname + ".cast_shadow", cast_shadow);
}

void Lobo::LoboLighting::setPointLight(LoboShader *render_shader,
                                       std::string lightname) {
    render_shader->setVec3(lightname + ".position", lightPos);
    render_shader->setVec3(lightname + ".lightColor", lightColor);
    render_shader->setFloat(lightname + ".constant", constant);
    render_shader->setFloat(lightname + ".linear", linear);
}

void Lobo::LoboLighting::setDirectionalLight(LoboShader *render_shader,
                                             std::string lightname) {
    direction = -lightPos;
    render_shader->setVec3(lightname + ".direction", direction);
    render_shader->setVec3(lightname + ".lightColor", lightColor);
    render_shader->setVec3(lightname + ".position", lightPos);
}

void Lobo::LoboLighting::setLightShadow(LoboShader *depth_shader, int lightid) {

    std::ostringstream stringStream;
    stringStream << "lightSpaceMatrix[" << lightid << "]";
    std::string lightname = stringStream.str();
    std::ostringstream stringStream2;
    stringStream2 << "shadowMap[" << lightid << "]";
    std::string shadowMapname = stringStream2.str();

    glm::mat4 lightProjection, lightView;
    glm::mat4 lightSpaceMatrix;
    float near_plane = 0.01f, far_plane = 20.0f;
    lightProjection =
        glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);
    Lobo::Camera *current_camera = Lobo::getCurrentCamera();
    float ratio = current_camera->view_port[2]/current_camera->view_port[3];
    if(this->light_type == 0)
    lightProjection = glm::perspective(
        (float)glm::radians(60.0), ratio, 0.001f, 100.0f);

    lightView = glm::lookAt(lightPos, glm::vec3(0.0), glm::vec3(0.0, 1.0, 0.0));
    lightSpaceMatrix = lightProjection * lightView;
    // render scene from light's point of view
    depth_shader->useProgram();
    depth_shader->setMat4(lightname, lightSpaceMatrix);
    depth_shader->setInt(shadowMapname, 6 + lightid);

    glActiveTexture(GL_TEXTURE6 + lightid);
    glBindTexture(GL_TEXTURE_2D, depthMap);
}

void Lobo::LoboLighting::setLightDepthShadow(LoboShader *depth_shader,
                                             int lightid) {
    glm::mat4 lightProjection, lightView;
    glm::mat4 lightSpaceMatrix;
    float near_plane = 0.01f, far_plane = 20.0f;
    lightProjection =
        glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);
    
    Lobo::Camera *current_camera = Lobo::getCurrentCamera();
    float ratio = current_camera->view_port[2]/current_camera->view_port[3];
    if(this->light_type == 0)
    lightProjection = glm::perspective(
        (float)glm::radians(60.0), ratio, 0.001f, 100.0f);
    
    lightView = glm::lookAt(lightPos, glm::vec3(0.0), glm::vec3(0.0, 1.0, 0.0));
    lightSpaceMatrix = lightProjection * lightView;
    // render scene from light's point of view
    depth_shader->useProgram();
    depth_shader->setMat4("lightSpaceMatrix", lightSpaceMatrix);
}

void Lobo::LoboLighting::initShadowMap() {
    glGenFramebuffers(1, &depthMapFBO);
    // create depth texture
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH,
                 SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = {1.0, 1.0, 1.0, 1.0};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    // attach depth texture as FBO's depth buffer
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
