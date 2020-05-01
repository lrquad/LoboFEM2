#include "LoboMesh.h"
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <experimental/filesystem>
#include <iostream>
#include "MeshControl.h"
#include "OpenGLutils/glfunctions.h"
#include "imgui.h"
#include "stb_image.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <omp.h>
#include "glm/gtx/euler_angles.hpp"
#include "Collision/CollisionDector/BVHCollisionDetector.h"

namespace fs = std::experimental::filesystem;

Lobo::LoboMesh::LoboMesh() { defaultValue(); }

Lobo::LoboMesh::LoboMesh(const char *filename, bool uniform)
{
    defaultValue();
    loadObj(filename, uniform);
}

Lobo::LoboMesh::~LoboMesh() { this->deleteGL(); }

void Lobo::LoboMesh::defaultValue()
{
    glinitialized = false;
    bufferNeedUpdate = false;
    position = glm::vec3(0.0);
    eular_angle = glm::vec3(0.0);
    this->scale = glm::vec3(1.0);
    start_show_material = 0;
    num_faces = 0;
    bvh_dectector = NULL;
    //omp_set_dynamic(0);  // Explicitly disable dynamic teams
    //omp_set_num_threads(12);  // Use 4 threads for all consecutive parallel regions
}

void Lobo::LoboMesh::drawImGui(bool *p_open)
{
    if (ImGui::CollapsingHeader(obj_file_name.c_str(),
                                ImGuiWindowFlags_NoCollapse))
    {
        if (ImGui::TreeNodeEx("Geometry##2", ImGuiWindowFlags_NoCollapse))
        {
            ImGui::Text("File name: %s ", obj_file_name.c_str());
            ImGui::Text("num vertices: %d; num faces: %d; num normals: %d",
                        attrib.vertices.size() / 3, num_faces,
                        attrib.normals.size() / 3);
            ImGui::Text("num texcoords: %d ", attrib.texcoords.size() / 2);
            ImGui::Text("num shapes: %d num materials: %d", shapes.size(),
                        materials.size());

            bool p_changed = ImGui::DragFloat3("drag position", &position.r,
                                               0.01f, -10.0f, 10.0f);
            bool r_changed = ImGui::DragFloat3(
                "drag eular_angle", &eular_angle.r, 1.0f, -360.0f, 360.0f);

            bool s_changed = ImGui::DragFloat3("drag scale", &scale.r, 0.1f, -100.0f, 100.0f);

            if (p_changed || r_changed || s_changed)
                this->updateRigidTransformation(position, eular_angle, scale);

            if (ImGui::Button("Reset position"))
            {
                this->resetVertice();
            };
            ImGui::TreePop();
            ImGui::Separator();
        }

        shader_config.drawImGui();

        if (ImGui::TreeNodeEx("Materials##2", ImGuiWindowFlags_NoCollapse))
        {
            ImGui::InputInt("Material start at", &start_show_material, 1, 5);
            if (start_show_material < 0)
            {
                start_show_material = 0;
            }

            if (materials.size() > 3)
                start_show_material %= materials.size() - 3;

            for (size_t i = start_show_material; i < start_show_material + 3;
                 i++)
            {
                if (i > materials.size() - 1)
                {
                    break;
                }
                ImGui::PushID(i);
                ImGui::Checkbox("use textures",
                                &material_buffer[i].use_diffuse_tex);
                ImGui::Checkbox("use normal texture",
                                &material_buffer[i].use_normal_tex);
                ImGui::Checkbox("use bump texture",
                                &material_buffer[i].use_bump_tex);

                ImGui::Text("mat[%d].diff_texname:%s", int(i),
                            materials[i].diffuse_texname.c_str());

                ImGui::ColorEdit3("ambient color", &(materials[i].ambient)[0]);
                ImGui::ColorEdit3("diffuse color", &(materials[i].diffuse)[0]);
                ImGui::ColorEdit3("specular color",
                                  &(materials[i].specular)[0]);
                ImGui::DragFloat("shininess", &materials[i].shininess, 0.05f,
                                 0.0, 100.0);

                ImGui::PopID();
            }
            ImGui::TreePop();
            ImGui::Separator();
        }
    }
}

void Lobo::LoboMesh::loadObj(const char *filename, bool uniform, bool verbose)
{
    std::string warn;
    std::string err;
    fs::path p = filename;
    std::string mtl_base_dir = p.parent_path().string();
    obj_file_name = p.filename().string();
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                filename, mtl_base_dir.c_str());

    // initial vertex color
    vertex_color.resize(attrib.vertices.size());
    ori_vertices.resize(attrib.vertices.size());
    std::fill(vertex_color.begin(), vertex_color.end(), 0.0);

    // Append `default` material
    materials.push_back(tinyobj::material_t());
    for (int i = 0; i < 3; i++)
    {
        materials.back().ambient[i] = 0.0;
        materials.back().diffuse[i] = 0.7;
        materials.back().specular[i] = 1.0;
    }

    num_faces = 0;
    for (int i = 0; i < shapes.size(); i++)
    {
        num_faces += shapes[i].mesh.indices.size() / 3;
    }

    if (!warn.empty())
    {
        std::cout << warn << std::endl;
    }

    if (!err.empty())
    {
        std::cerr << err << std::endl;
    }

    if (!ret)
    {
        std::cout << "failed to load obj file..." << ret << std::endl;
        std::cout << filename << std::endl;
    }
    if (uniform)
        uniformMesh();

    memcpy(ori_vertices.data(), attrib.vertices.data(),
           sizeof(float) * attrib.vertices.size());
}

void Lobo::LoboMesh::uniformMesh()
{
    tinyobj::attrib_t output = attrib;
    Eigen::Vector3d origin_center;
    Lobo::centerTinyAttribute(attrib, output, origin_center);
    attrib = output;
    Lobo::uniformTinyAttribute(attrib, output);
    attrib = output;
    Lobo::updateSmoothNorm(attrib, shapes);
}

void Lobo::LoboMesh::initialGL()
{
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    int num_shapes = shapes.size();
    shape_buffer.resize(num_shapes);

    for (int i = 0; i < num_shapes; i++)
    {
        shape_buffer[i].size_per_vertex = 11;
        int buffersize =
            shapes[i].mesh.indices.size() * (shape_buffer[i].size_per_vertex);
        shape_buffer[i].vb.resize(buffersize); // pos(3 float) normal(3 float),
                                               // tex(2 float), color(3 float)
        // convert buffer
        updateShapeArrayBuffer(i);

        shape_buffer[i].material_id = (shapes)[i].mesh.material_ids[0];
        if (shape_buffer[i].material_id == -1)
        {
            shape_buffer[i].material_id = materials.size() - 1;
        }

        glGenBuffers(1, &shape_buffer[i].VBO);
        glBindBuffer(GL_ARRAY_BUFFER, shape_buffer[i].VBO);
        glBufferData(GL_ARRAY_BUFFER, shape_buffer[i].vb.size() * sizeof(float),
                     &(shape_buffer[i].vb)[0], GL_DYNAMIC_DRAW);
        setPositionAttribute(0, 3, 11, 0);
        setPositionAttribute(1, 3, 11, 3);
        setPositionAttribute(2, 2, 11, 6);
        setPositionAttribute(3, 3, 11, 8);
        // load texture
    }

    material_buffer.resize(materials.size());
    for (size_t m = 0; m < materials.size(); m++)
    {
        tinyobj::material_t *mp = &materials[m];
        // apply default ambient
        mp->ambient[0] = 0.0;
        mp->ambient[1] = 0.0;
        mp->ambient[2] = 0.0;

        material_buffer[m].has_diffuse_tex = false;
        material_buffer[m].use_diffuse_tex = false;
        material_buffer[m].has_emissive_tex = false;
        material_buffer[m].has_bump_tex = false;
        material_buffer[m].has_normal_tex = false;
        material_buffer[m].use_bump_tex = false;
        material_buffer[m].use_normal_tex = false;

        if (mp->diffuse_texname.length() > 0)
        {
            material_buffer[m].has_diffuse_tex = true;
            material_buffer[m].use_diffuse_tex = true;
            // diffuse texture
            // glGenTextures(1, &material_buffer[m].diffuse_texid);
            int channels = bindTextureBuffer(mp->diffuse_texname.c_str(),
                                             material_buffer[m].diffuse_texid);
            material_buffer[m].diffuse_texname = mp->diffuse_texname;
        }
        if (mp->emissive_texname.length() > 0)
        {
            material_buffer[m].has_emissive_tex = true;
            material_buffer[m].use_diffuse_tex = true;
            // glGenTextures(1, &material_buffer[m].emissive_texid);
            int channels = bindTextureBuffer(mp->emissive_texname.c_str(),
                                             material_buffer[m].emissive_texid);
            material_buffer[m].emissive_texname = mp->emissive_texname;
        }
        if (mp->normal_texname.length() > 0)
        {
            material_buffer[m].has_normal_tex = true;
            material_buffer[m].use_normal_tex = true;
            int channels = bindTextureBuffer(mp->normal_texname.c_str(),
                                             material_buffer[m].normal_texid);
            material_buffer[m].normal_texname = mp->normal_texname;
        }
        if (mp->bump_texname.length() > 0)
        {
            material_buffer[m].has_bump_tex = true;
            material_buffer[m].use_bump_tex = true;
            int channels = bindTextureBuffer(mp->bump_texname.c_str(),
                                             material_buffer[m].bump_texid);
            material_buffer[m].bump_texname = mp->bump_texname;
        }
    }

    glBindVertexArray(0);
    glinitialized = true;
}

void Lobo::LoboMesh::updateShapeArrayBufferVertices(int shape_id)
{
    int size_per_vertex = shape_buffer[shape_id].size_per_vertex;

#pragma omp parallel for
    for (int j = 0; j < shapes[shape_id].mesh.indices.size(); j++)
    {
        int vid = shapes[shape_id].mesh.indices[j].vertex_index;
        shape_buffer[shape_id].vb[j * size_per_vertex + 0] =
            attrib.vertices[vid * 3 + 0];
        shape_buffer[shape_id].vb[j * size_per_vertex + 1] =
            attrib.vertices[vid * 3 + 1];
        shape_buffer[shape_id].vb[j * size_per_vertex + 2] =
            attrib.vertices[vid * 3 + 2];

        int nid = shapes[shape_id].mesh.indices[j].normal_index;
        if (nid != -1)
        {
            shape_buffer[shape_id].vb[j * size_per_vertex + 3] =
                attrib.normals[nid * 3 + 0];
            shape_buffer[shape_id].vb[j * size_per_vertex + 4] =
                attrib.normals[nid * 3 + 1];
            shape_buffer[shape_id].vb[j * size_per_vertex + 5] =
                attrib.normals[nid * 3 + 2];
        }
    }
}

void Lobo::LoboMesh::updateShapeArrayBufferIndices(int shape_id) {}

void Lobo::LoboMesh::updateShapeArrayBuffer(int shape_id)
{
    int size_per_vertex = shape_buffer[shape_id].size_per_vertex;

    for (int j = 0; j < shapes[shape_id].mesh.indices.size(); j++)
    {
        int vid = shapes[shape_id].mesh.indices[j].vertex_index;
        shape_buffer[shape_id].vb[j * size_per_vertex + 0] =
            attrib.vertices[vid * 3 + 0];
        shape_buffer[shape_id].vb[j * size_per_vertex + 1] =
            attrib.vertices[vid * 3 + 1];
        shape_buffer[shape_id].vb[j * size_per_vertex + 2] =
            attrib.vertices[vid * 3 + 2];

        shape_buffer[shape_id].vb[j * size_per_vertex + 8] =
            vertex_color[vid * 3 + 0];
        shape_buffer[shape_id].vb[j * size_per_vertex + 9] =
            vertex_color[vid * 3 + 1];
        shape_buffer[shape_id].vb[j * size_per_vertex + 10] =
            vertex_color[vid * 3 + 2];

        int nid = shapes[shape_id].mesh.indices[j].normal_index;

        if (nid != -1)
        {
            shape_buffer[shape_id].vb[j * size_per_vertex + 3] =
                attrib.normals[nid * 3 + 0];
            shape_buffer[shape_id].vb[j * size_per_vertex + 4] =
                attrib.normals[nid * 3 + 1];
            shape_buffer[shape_id].vb[j * size_per_vertex + 5] =
                attrib.normals[nid * 3 + 2];
        }

        int tid = shapes[shape_id].mesh.indices[j].texcoord_index;

        if (tid == -1)
        {
            shape_buffer[shape_id].vb[j * size_per_vertex + 6] = 0;
            shape_buffer[shape_id].vb[j * size_per_vertex + 7] = 0;
            // continue;
        }
        else
        {
            shape_buffer[shape_id].vb[j * size_per_vertex + 6] =
                attrib.texcoords[tid * 2 + 0];
            shape_buffer[shape_id].vb[j * size_per_vertex + 7] =
                attrib.texcoords[tid * 2 + 1];
        }
    }
}

void Lobo::LoboMesh::updateGLbuffer()
{
    // check if the buffer is already updated
    if (bufferNeedUpdate)
    {
        Lobo::updateSmoothNorm(attrib, shapes);
        glBindVertexArray(VAO);
        int num_shapes = shapes.size();
        for (int i = 0; i < num_shapes; i++)
        {
            // convert buffer
            updateShapeArrayBufferVertices(i);
            glBindBuffer(GL_ARRAY_BUFFER, shape_buffer[i].VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            shape_buffer[i].vb.size() * sizeof(float),
                            &(shape_buffer[i].vb)[0]);
            // setPositionAttribute(0, 3, 11, 0);
            // setPositionAttribute(1, 3, 11, 3);
            // setPositionAttribute(2, 2, 11, 6);
            // setPositionAttribute(3, 3, 11, 8);
            // load texture
        }
    }
    bufferNeedUpdate = false;
}

void Lobo::LoboMesh::paintGL(LoboShader *shader)
{
    // update vertex buffer before painting

    shader_config.setShader(shader);
    if (shader_config.visiable == false)
    {
        return;
    }
    // glBindTexture(GL_TEXTURE_2D, texture);
    updateGLbuffer();

    glBindVertexArray(VAO);
    for (int i = 0; i < shape_buffer.size(); i++)
    {
        int materia_id = shape_buffer[i].material_id;

        shader->setInt("material.diffuse_tex", 0);
        shader->setInt("material.emissive_tex", 1);
        shader->setInt("material.normal_tex", 2);
        shader->setInt("material.bump_tex", 3);

        Lobo::activeTexture(material_buffer[i].has_diffuse_tex == true &&
                                material_buffer[i].use_diffuse_tex == true,
                            0, material_buffer[materia_id].diffuse_texid);
        Lobo::activeTexture(material_buffer[i].has_emissive_tex == true &&
                                material_buffer[i].use_diffuse_tex == true,
                            1, material_buffer[materia_id].emissive_texid);
        Lobo::activeTexture(material_buffer[i].has_normal_tex == true &&
                                material_buffer[i].use_normal_tex == true,
                            2, material_buffer[materia_id].normal_texid);
        Lobo::activeTexture(material_buffer[i].has_bump_tex == true &&
                                material_buffer[i].use_bump_tex == true,
                            3, material_buffer[materia_id].bump_texid);

        glm::vec3 diffuse_color = glm::vec3(materials[materia_id].diffuse[0],
                                            materials[materia_id].diffuse[1],
                                            materials[materia_id].diffuse[2]);
        glm::vec3 ambient_color = glm::vec3(materials[materia_id].ambient[0],
                                            materials[materia_id].ambient[1],
                                            materials[materia_id].ambient[2]);
        glm::vec3 specular_color = glm::vec3(materials[materia_id].specular[0],
                                             materials[materia_id].specular[1],
                                             materials[materia_id].specular[2]);

        // shader->setVec3("diffuse_color", color);
        shader->setBool("useDiffuseTex",
                        material_buffer[i].use_diffuse_tex &&
                            material_buffer[i].has_diffuse_tex);
        shader->setBool("useNormalTex", material_buffer[i].use_normal_tex &&
                                            material_buffer[i].has_normal_tex);
        shader->setBool("useBumpTex", material_buffer[i].use_bump_tex &&
                                          material_buffer[i].has_bump_tex);

        shader->setVec3("material.ambient", ambient_color);
        shader->setVec3("material.diffuse", diffuse_color);
        shader->setVec3("material.specular", specular_color);
        shader->setFloat("material.shininess", 32.0f);

        glBindBuffer(GL_ARRAY_BUFFER, shape_buffer[i].VBO);
        setPositionAttribute(0, 3, 11, 0);
        setPositionAttribute(1, 3, 11, 3);
        setPositionAttribute(2, 2, 11, 6);
        setPositionAttribute(3, 3, 11, 8);

        glDrawArrays(
            GL_TRIANGLES, 0,
            shape_buffer[i].vb.size() / shape_buffer[i].size_per_vertex);
    }
    // glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void Lobo::LoboMesh::deleteGL()
{
    if (glinitialized == true)
    {
        glDeleteVertexArrays(1, &VAO);
        for (int i = 0; i < shape_buffer.size(); i++)
        {
            glDeleteBuffers(1, &shape_buffer[i].VBO);
        }

        glinitialized = false;
    }
}

void Lobo::LoboMesh::setPosition(std::vector<float> position_)
{
    position[0] = position_[0];
    position[1] = position_[1];
    position[2] = position_[2];
    updateRigidTransformation(position, eular_angle, scale);
}

void Lobo::LoboMesh::setAngle(std::vector<float> angle_)
{
    eular_angle[0] = angle_[0];
    eular_angle[1] = angle_[1];
    eular_angle[2] = angle_[2];
    updateRigidTransformation(position, eular_angle, scale);
}

void Lobo::LoboMesh::setScale(std::vector<float> scale_)
{
    scale[0] = scale_[0];
    scale[1] = scale_[1];
    scale[2] = scale_[2];
    updateRigidTransformation(position, eular_angle, scale);
}

void Lobo::LoboMesh::updateRigidTransformation(glm::vec3 position,
                                               glm::vec3 eular_angle, glm::vec3 scale)
{
    glm::mat4 rotation = glm::eulerAngleXYZ(glm::radians(eular_angle[0]),
                                            glm::radians(eular_angle[1]),
                                            glm::radians(eular_angle[2]));

    for (int i = 0; i < ori_vertices.size() / 3; i++)
    {
        glm::vec4 ori_p =
            glm::vec4(ori_vertices[i * 3 + 0] * scale[0], ori_vertices[i * 3 + 1] * scale[1],
                      ori_vertices[i * 3 + 2] * scale[2], 1.0);
        glm::vec4 cur_p = rotation * ori_p;

        attrib.vertices[i * 3 + 0] = cur_p[0] + position[0];
        attrib.vertices[i * 3 + 1] = cur_p[1] + position[1];
        attrib.vertices[i * 3 + 2] = cur_p[2] + position[2];
    }

    bufferNeedUpdate = true;
    updateGeometryInfo();
}

void Lobo::LoboMesh::updateVertices(float *newPosition)
{
    memcpy(attrib.vertices.data(), newPosition,
           sizeof(float) * attrib.vertices.size());
    bufferNeedUpdate = true;
    updateGeometryInfo();
}

void Lobo::LoboMesh::updateVertices(double *newPosition)
{
    for (int i = 0; i < attrib.vertices.size(); i++)
    {
        attrib.vertices[i] = newPosition[i];
    }
    bufferNeedUpdate = true;
    updateGeometryInfo();
}

void Lobo::LoboMesh::updateVerticesColor(double *newColor)
{
    
}

void Lobo::LoboMesh::getCurVertices(float *outPosition)
{
    memcpy(outPosition, attrib.vertices.data(),
           sizeof(float) * attrib.vertices.size());
}

void Lobo::LoboMesh::getCurVertices(double *outPosition)
{
    for (int i = 0; i < attrib.vertices.size(); i++)
    {
        outPosition[i] = attrib.vertices[i];
    }
}

void Lobo::LoboMesh::getFaceIndices(int *indices)
{
    int offset = 0;
    for (int i = 0; i < shapes.size(); i++)
    {
        for (int j = 0; j < shapes[i].mesh.indices.size(); j++)
        {
            indices[offset] = shapes[i].mesh.indices[j].vertex_index;
            offset++;
        }
    }
}

void Lobo::LoboMesh::resetVertice()
{
    memcpy(attrib.vertices.data(), ori_vertices.data(),
           sizeof(float) * attrib.vertices.size());
    bufferNeedUpdate = true;
}

void Lobo::LoboMesh::updateGeometryInfo()
{
    if (bvh_dectector != NULL)
    {
        bvh_dectector->updateCollisionShape();
    }
}