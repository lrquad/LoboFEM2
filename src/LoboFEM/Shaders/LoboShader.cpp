#include "LoboShader.h"
#include <fstream>
#include <iostream>
#include "imgui.h"

Lobo::LoboShaderConfig::LoboShaderConfig()
{
    wireframe_mode = false;
    flat_mode = true;
    visiable = true;
    vertex_color_mode=false;
    cast_shadow = true;
    use_blinn = true;
    point_mode=false;
}

Lobo::LoboShaderConfig::~LoboShaderConfig()
{

}

void Lobo::LoboShaderConfig::drawImGui(bool *p_open) {
    if (ImGui::TreeNodeEx("Shader Configuration##2",ImGuiWindowFlags_NoCollapse)) {
        ImGui::Checkbox("wireframe_mode", &wireframe_mode);
        ImGui::Checkbox("point_mode",&point_mode);
        ImGui::Checkbox("flat_mode", &flat_mode);
        ImGui::Checkbox("use_blinn",&use_blinn);
        ImGui::Checkbox("visiable",&visiable);
        ImGui::Checkbox("vertex_color_mode",&vertex_color_mode);
        ImGui::Checkbox("cast_shadow",&cast_shadow);
        ImGui::TreePop();
        ImGui::Separator();
    }
}

void Lobo::LoboShaderConfig::setShader(LoboShader* shader)
{
    glPointSize(1.0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    if (wireframe_mode == true) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    if(point_mode==true) 
    {
        glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
        glPointSize(10.0);
    }

    shader->setBool("useFlatNormal",flat_mode);
    shader->setBool("vertex_color_mode",vertex_color_mode);
    shader->setBool("use_blinn",use_blinn);
}

std::string Lobo::LoboShader::readFile(const char *filename) {
    std::string line, text;
    std::ifstream in(filename);
    while (std::getline(in, line)) {
        text += line + "\n";
    }
    text += "\0";
    return text;
}

void Lobo::LoboShader::drawImGui(bool *p_open) {
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    ImGui::Begin("Shader control", p_open, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Output color");
    ImGui::ColorEdit3("clear color", (float *)&clear_color);
    ImGui::End();
}

void Lobo::LoboShader::loadShader() {
    std::string vertexShaderSource = readFile("./shaders/simplevertex.glsl");
    std::string fragmentShaderSource = readFile("./shaders/simplefrag.glsl");

    this->loadShaderStream(vertexShaderSource.c_str(),
                           fragmentShaderSource.c_str());
}

void Lobo::LoboShader::loadShaderFile(const char *vsfile, const char *vffile) {
    std::string vertexShaderSource = readFile(vsfile);
    std::string fragmentShaderSource = readFile(vffile);

    this->loadShaderStream(vertexShaderSource.c_str(),
                           fragmentShaderSource.c_str());
}

void Lobo::LoboShader::loadShaderStream(const char *vertexShaderSource,
                                        const char *fragmentShaderSource) {
    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertex_shader);
    int success;
    char infoLog[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertex_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
                  << infoLog << std::endl;
    }

    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragment_shader);
    // check for shader compile errors
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragment_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
                  << infoLog << std::endl;
    }

    // link shaders
    shader_program = glCreateProgram();
    glAttachShader(shader_program, vertex_shader);
    glAttachShader(shader_program, fragment_shader);
    glLinkProgram(shader_program);
    // check for linking errors
    glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shader_program, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
                  << infoLog << std::endl;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
}

void Lobo::LoboShader::useProgram() { glUseProgram(shader_program); }

void Lobo::LoboShader::setBool(const std::string &name, bool value) const {
    glUniform1i(glGetUniformLocation(shader_program, name.c_str()), (int)value);
}
void Lobo::LoboShader::setInt(const std::string &name, int value) const {
    glUniform1i(glGetUniformLocation(shader_program, name.c_str()), value);
}
void Lobo::LoboShader::setFloat(const std::string &name, float value) const {
    glUniform1f(glGetUniformLocation(shader_program, name.c_str()), value);
}

void Lobo::LoboShader::setFloat4(const std::string &name, float v0, float v1,
                                 float v2, float v3) const {
    glUniform4f(glGetUniformLocation(shader_program, name.c_str()), v0, v1, v2,
                v3);
}
