#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>
#include <glm/glm.hpp>

namespace Lobo
{



class LoboShader
{
public:
    LoboShader(){};
    ~LoboShader(){};

    virtual void drawImGui(bool *p_open = NULL);

    virtual void loadShaderFile(const char *vertexShaderfile, const char *fragmentShaderfile);
    virtual void loadShaderStream(const char *vertexShaderSource, const char *fragmentShaderSource);
    virtual void loadShader();
    virtual void useProgram();


    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setFloat4(const std::string &name, float v0, float v1, float v2, float v3) const;

    // utility uniform functions
    // ------------------------------------------------------------------------
    void setVec2(const std::string &name, const glm::vec2 &value) const
    { 
        glUniform2fv(glGetUniformLocation(shader_program, name.c_str()), 1, &value[0]); 
    }
    void setVec2(const std::string &name, float x, float y) const
    { 
        glUniform2f(glGetUniformLocation(shader_program, name.c_str()), x, y); 
    }
    // ------------------------------------------------------------------------
    void setVec3(const std::string &name, const glm::vec3 &value) const
    { 
        glUniform3fv(glGetUniformLocation(shader_program, name.c_str()), 1, &value[0]); 
    }
    void setVec3(const std::string &name, float x, float y, float z) const
    { 
        glUniform3f(glGetUniformLocation(shader_program, name.c_str()), x, y, z); 
    }
    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::vec4 &value) const
    { 
        glUniform4fv(glGetUniformLocation(shader_program, name.c_str()), 1, &value[0]); 
    }
    void setVec4(const std::string &name, float x, float y, float z, float w) 
    { 
        glUniform4f(glGetUniformLocation(shader_program, name.c_str()), x, y, z, w); 
    }
    // ------------------------------------------------------------------------
    void setMat2(const std::string &name, const glm::mat2 &mat) const
    {
        glUniformMatrix2fv(glGetUniformLocation(shader_program, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat3(const std::string &name, const glm::mat3 &mat) const
    {
        glUniformMatrix3fv(glGetUniformLocation(shader_program, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat4(const std::string &name, const glm::mat4 &mat) const
    {
        glUniformMatrix4fv(glGetUniformLocation(shader_program, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }


protected:
    virtual std::string readFile(const char *filename);

    
    int vertex_shader;
    int fragment_shader;
    int shader_program;
};

class LoboShaderConfig
{
    public:
    LoboShaderConfig();
    ~LoboShaderConfig();
    virtual void drawImGui(bool *p_open = NULL);
    virtual void setShader(LoboShader* shader);

    bool vertex_color_mode;
    bool wireframe_mode;
    bool point_mode;
    bool flat_mode;
    bool visiable;
    bool cast_shadow;
    bool use_blinn;
};

} // namespace Lobo
