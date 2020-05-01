#pragma once
#include "Shaders/LoboShader.h"
#include <glm/glm.hpp>
#include <map>
#include "ObjLoader/tiny_obj_loader.h"
#include <Eigen/Dense>
#include <fstream>

class BVHCollisionDetector;

namespace Lobo {

inline void exportSimpleObj(const char* filename, Eigen::MatrixXd& tri_vertices,
                            Eigen::MatrixXi& tri_faces) {
    std::ofstream output(filename);
    for (int i = 0; i < tri_vertices.rows(); i++) {
        output << "v ";
        for (int j = 0; j < 3; j++) {
            output << tri_vertices.data()[j * tri_vertices.rows() + i] << " ";
        }
        output << std::endl;
    }

    for (int i = 0; i < tri_faces.rows(); i++) {
        output << "f ";
        for (int j = 0; j < 3; j++) {
            output << tri_faces.data()[j * tri_faces.rows() + i]+1 << " ";
        }
        output << std::endl;
    }

    output.close();
}

struct ShapeBuffer {
    unsigned int VBO;  // vertex array buffer

    std::vector<float> vb;
    int size_per_vertex;
    int material_id;
};

struct MaterialBuffer {
    unsigned int diffuse_texid;   // vertex array buffer
    unsigned int emissive_texid;  // vertex array buffer
    unsigned int normal_texid;  // vertex array buffer
    unsigned int bump_texid;

    std::string diffuse_texname;
    std::string emissive_texname;
    std::string normal_texname;
    std::string bump_texname;

    // std::map<std::string,unsigned int> disffues_tex;
    bool has_diffuse_tex;
    bool has_emissive_tex;
    bool has_normal_tex;
    bool has_bump_tex;


    bool use_diffuse_tex;
    bool use_normal_tex;
    bool use_bump_tex;

};

class LoboMesh {
   public:
    LoboMesh();
    LoboMesh(const char* filename, bool uniform = true);
    ~LoboMesh();

    virtual void loadObj(const char* filename, bool uniform,
                         bool verbose = false);
    virtual void uniformMesh();
    virtual void drawImGui(bool* p_open = NULL);

    virtual void initialGL();
    virtual void updateGLbuffer();
    virtual void paintGL(LoboShader* shader);
    virtual void deleteGL();

    std::string obj_file_name;

    tinyobj::attrib_t attrib;
    std::vector<float> vertex_color;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    int num_faces;

    // GL
    std::vector<ShapeBuffer> shape_buffer;
    std::vector<MaterialBuffer> material_buffer;

    virtual void updateShapeArrayBuffer(int shape_id);
    virtual void updateShapeArrayBufferIndices(int shape_id);

    virtual void updateShapeArrayBufferVertices(int shape_id);

    // dynamic interface
    std::vector<float> ori_vertices;  // 3n

    virtual void setPosition(std::vector<float> position_);
    virtual void setAngle(std::vector<float> angle_);
    virtual void setScale(std::vector<float> scale);



    virtual void updateRigidTransformation(glm::vec3 position,
                                           glm::vec3 eular_angle,glm::vec3 scale);

    virtual void updateVertices(float* newPosition);
    virtual void updateVertices(double* newPosition);
    virtual void updateVerticesColor(double* newColor);

    virtual void getCurVertices(float* outPosition);
    virtual void getCurVertices(double* outPosition);
    virtual void getFaceIndices(int* indices);

    virtual void updateGeometryInfo();

    virtual void resetVertice();
    Lobo::LoboShaderConfig shader_config;

    BVHCollisionDetector* bvh_dectector;

   protected:
    // imgui
    unsigned int VAO;  // vertex array buffer

    int start_show_material;

    bool glinitialized;
    bool bufferNeedUpdate;
    // mesh rigid configure;
    glm::vec3 position;
    glm::vec3 eular_angle;  //"xyz"
    glm::vec3 scale;  //"xyz"
    

    virtual void defaultValue();
};

}  // namespace Lobo