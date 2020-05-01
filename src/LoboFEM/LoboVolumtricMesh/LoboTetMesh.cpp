#include "LoboTetMesh.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/project.h>
#include <igl/barycentric_coordinates.h>
#include <igl/in_element.h>
#include <igl/AABB.h>
#include <igl/per_vertex_normals.h>
#include <igl/tet_tet_adjacency.h>

// boost headers

#include "Functions/EigenMatrixIO.h"
#include "Functions/computeTriangle.h"
#include "Functions/findElementInVector.h"

#include "LoboDynamic/LoboDynamicScene.h"
#include "LoboMesh/LoboMesh.h"
#include "imgui.h"
#include "LoboImGui/cpp/imgui_stdlib.h"
#include "OpenGLutils/glfunctions.h"
#include "OpenGLutils/LoboCamera.h"
#include "Utils/glmEigenConverter.h"
#include "Utils/glmMyFunctions.h"

Lobo::LoboTetMesh::LoboTetMesh() {
    initializedGL = false;
    render_normals = false;
    tetgen_command = "pq1.414";
    status_flags = 0;
    default_material.ambient[0] = 0.0;
    default_material.ambient[1] = 0.0;
    default_material.ambient[2] = 0.0;
    default_material.diffuse[0] = 1.0;
    default_material.diffuse[1] = 0.3;
    default_material.diffuse[2] = 0.3;
    default_material.specular[0] = 0.8;
    default_material.specular[1] = 0.8;
    default_material.specular[2] = 0.8;

    shader_config.wireframe_mode = false;
    shader_config.vertex_color_mode = true;
    lobomesh_binding = NULL;

    mesh_total_volume = 0.0;
    numElementVertices = 4;

    clicked_face = 0;
}

Lobo::LoboTetMesh::~LoboTetMesh() {}

void Lobo::LoboTetMesh::drawImGui(bool *p_open) {
    ImGuiIO &io = ImGui::GetIO();
    if (ImGui::CollapsingHeader(filebase.c_str(),
                                ImGuiWindowFlags_NoCollapse)) {
        // static char str0[128] = tetgen_command;
        // ImGui::InputText("Tet file base", str0, IM_ARRAYSIZE(str0));
        // ImGui::InputText("Tet gen command", tetgen_command.data(),
        // IM_ARRAYSIZE(str0));
        ImGui::Text("InitialGl %s", status_flags & TetMeshStatusFlags_initialGL
                                        ? "true"
                                        : "false");
        ImGui::Text("datasize changed %s",
                    status_flags & TetMeshStatusFlags_datasizeUpdated
                        ? "true"
                        : "false");
        ImGui::Text("Tetgened %s", status_flags & TetMeshStatusFlags_tetgened
                                       ? "true"
                                       : "false");
        ImGui::Text("Loadedgen %s", status_flags & TetMeshStatusFlags_loadtet
                                        ? "true"
                                        : "false");

        ImGui::InputText("Tet file base ", &filebase);
        ImGui::InputText("tetgen_command ", &tetgen_command);

        if (ImGui::Button("Generate Tet")) {
            this->generateTet(tetgen_command.c_str());
        };
        ImGui::SameLine();
        if (ImGui::Button("Save Tet")) {
            exportTetMesh();
        };
        ImGui::SameLine();
        if (ImGui::Button("load Tet")) {
            loadTetMesh();
        };
        if (ImGui::Button("Save Constraints")) {
            exportConstrainedVertices(
                Lobo::getPath("constraints/constrainedDoFs.txt").c_str());
        }

        if (status_flags &
            (TetMeshStatusFlags_tetgened | TetMeshStatusFlags_loadtet)) {
            ImGui::Text("tet_vertice %d tet_num_tet %d", tet_vertice.rows() / 3,
                        tet_indices.rows() / 4);

            ImGui::Text("Mesh volume: %.3f m^3", mesh_total_volume);
        }

        if (ImGui::Button("hide trimesh")) {
            if (lobomesh_binding != NULL) {
                lobomesh_binding->shader_config.visiable =
                    !lobomesh_binding->shader_config.visiable;
            }
        }

        if (ImGui::Button("hide normal")) {
            render_normals = !render_normals;
        }

        if (ImGui::TreeNodeEx("TetMeshConfig##1",
                              ImGuiWindowFlags_NoCollapse)) {
            ImGui::Checkbox("usebinary", &usebinary);
            ImGui::TreePop();
            ImGui::Separator();
        }

        shader_config.drawImGui();

        if (ImGui::TreeNodeEx("Materials##3")) {
            ImGui::ColorEdit3("ambient color", &(default_material.ambient)[0]);
            ImGui::ColorEdit3("diffuse color", &(default_material.diffuse)[0]);
            ImGui::ColorEdit3("specular color",
                              &(default_material.specular)[0]);
            ImGui::DragFloat("shininess", &default_material.shininess, 0.05f,
                             0.0, 100.0);
            ImGui::TreePop();
            ImGui::Separator();
        }
    }

    // mouse click
    if (ImGui::IsMouseClicked(1) && io.KeysDownDuration[341] >= 0.0f) {
        mouseClicked();
    }

    if (ImGui::IsMouseDragging(1) && io.KeysDownDuration[340] >= 0.0f) {
        mouseRectSelect();
    }

    if (ImGui::IsMouseDragging(1) && io.KeysDownDuration[341] >= 0.0f) {
        mouseDrag();
    }

    if (ImGui::IsMouseReleased(1)) {
        tet_vertice_force.setZero();
    }
}

void Lobo::LoboTetMesh::mouseRectSelect() {
    if (status_flags &
        (TetMeshStatusFlags_tetgened | TetMeshStatusFlags_loadtet)) {
        ImGuiIO &io = ImGui::GetIO();
        Lobo::Camera *current_camera = Lobo::getCurrentCamera();
        Eigen::Vector4f view_port =
            Lobo::GLM_2_E<float, 4>(current_camera->view_port);
        Eigen::Matrix4f view_m =
            Lobo::GLM_2_E<float, 4>(current_camera->view_matrix);
        Eigen::Matrix4f project_m =
            Lobo::GLM_2_E<float, 4>(current_camera->projection_matrix);
        Eigen::MatrixXf P;
        igl::project(tet_vertice_col, view_m, project_m, view_port, P);
        glm::vec4 mouse_rect(io.MouseClickedPos[1].x,
                             view_port.data()[3] - io.MouseClickedPos[1].y,
                             io.MousePos.x,
                             view_port.data()[3] - io.MousePos.y);
        for (int i = 0; i < P.rows(); i++) {
            if (Lobo::inRect(mouse_rect, P.data()[i], P.data()[P.rows() + i])) {
                setTetVetAttriColor(i, 0.0, 0.0, 1.0);
                vertices_flags[i] = 1;
            }
        }
    }
}

void Lobo::LoboTetMesh::mouseClicked() {
    if (status_flags &
        (TetMeshStatusFlags_tetgened | TetMeshStatusFlags_loadtet)) {
        ImGuiIO &io = ImGui::GetIO();
        Lobo::Camera *current_camera = Lobo::getCurrentCamera();
        Eigen::Vector4f view_port =
            Lobo::GLM_2_E<float, 4>(current_camera->view_port);
        Eigen::Matrix4f view_m =
            Lobo::GLM_2_E<float, 4>(current_camera->view_matrix);
        Eigen::Matrix4f project_m =
            Lobo::GLM_2_E<float, 4>(current_camera->projection_matrix);

        Eigen::Vector3f bc;
        int fid;
        // Cast a ray in the view direction starting from the mouse position
        double x = io.MousePos.x;
        double y = view_port.data()[3] - io.MousePos.y;

        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view_m, project_m,
                                     view_port, tet_vertice_col, tet_faces_col,
                                     fid, bc)) {
            for (int i = 0; i < 3; i++) {
                int vid = tet_faces.data()[fid * 3 + i];
                setTetVetAttriColor(vid, 1.0, 0.0, 0.0);
                if (i == 0) {
                    std::cout << "vid " << vid << std::endl;
                }
            }
            clicked_face = fid;
        }
        std::cout << "fid " << clicked_face << std::endl;
    }
}

void Lobo::LoboTetMesh::mouseDrag() {
    if (status_flags &
        (TetMeshStatusFlags_tetgened | TetMeshStatusFlags_loadtet)) {
        ImGuiIO &io = ImGui::GetIO();
        Lobo::Camera *current_camera = Lobo::getCurrentCamera();
        Eigen::Vector4f view_port =
            Lobo::GLM_2_E<float, 4>(current_camera->view_port);
        Eigen::Matrix4f view_m =
            Lobo::GLM_2_E<float, 4>(current_camera->view_matrix);
        Eigen::Matrix4f project_m =
            Lobo::GLM_2_E<float, 4>(current_camera->projection_matrix);

        Eigen::Vector3f bc;
        int fid;

        double x = io.MousePos.x;
        double y = view_port.data()[3] - io.MousePos.y;

        Eigen::Vector3f face_center = Eigen::Vector3f::Zero();
        for (int i = 0; i < 3; i++) {
            int vid = tet_faces.data()[clicked_face * 3 + i];
            for (int j = 0; j < 3; j++) {
                face_center.data()[j] += tet_vertice.data()[vid * 3 + j];
            }
        }

        face_center /= 3.0;
        Eigen::Vector3f screen_pose;
        screen_pose = igl::project(face_center, view_m, project_m, view_port);

        Eigen::Vector3f cur_screen_pose = screen_pose;
        cur_screen_pose.data()[0] = x;
        cur_screen_pose.data()[1] = y;
        Eigen::Vector3f cur_scene_pose;
        igl::unproject(cur_screen_pose, view_m, project_m, view_port,
                       cur_scene_pose);

        tet_vertice_force.setZero();
        for (int i = 0; i < 3; i++) {
            int vid = tet_faces.data()[clicked_face * 3 + i];
            for (int j = 0; j < 3; j++) {
                tet_vertice_force.data()[vid * 3 + j] +=
                    (cur_scene_pose.data()[j] - face_center.data()[j])*100.0;
            }
        }
    }
}

void Lobo::LoboTetMesh::paintGL(LoboShader *shader) {
    if (!(status_flags & TetMeshStatusFlags_initialGL)) {
        return;
    }
    if (status_flags & TetMeshStatusFlags_datasizeUpdated) {
        // need update
        updateGL();
    }

    shader_config.setShader(shader);
    if (shader_config.visiable == false) {
        return;
    }

    shader->setBool("useDiffuseTex", false);
    shader->setBool("useNormalTex", false);
    shader->setBool("useBumpTex", false);

    glm::vec3 diffuse_color =
        glm::vec3(default_material.diffuse[0], default_material.diffuse[1],
                  default_material.diffuse[2]);
    glm::vec3 ambient_color =
        glm::vec3(default_material.ambient[0], default_material.ambient[1],
                  default_material.ambient[2]);
    glm::vec3 specular_color =
        glm::vec3(default_material.specular[0], default_material.specular[1],
                  default_material.specular[2]);
    // shader->setVec3("diffuse_color", color);
    shader->setBool("useDiffuseTex", false);
    shader->setVec3("material.ambient", ambient_color);
    shader->setVec3("material.diffuse", diffuse_color);
    shader->setVec3("material.specular", specular_color);
    shader->setFloat("material.shininess", 32.0f);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    sizeof(float) * tet_vertice_attri.size(),
                    tet_vertice_attri.data());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    setPositionAttribute(0, 3, 11, 0);
    setPositionAttribute(1, 3, 11, 3);
    setPositionAttribute(2, 2, 11, 6);
    setPositionAttribute(3, 3, 11, 8);

    glDrawElements(GL_TRIANGLES, tet_faces_glint.size(), GL_UNSIGNED_INT, 0);

    if (status_flags & TetMeshStatusFlags_precomputed && render_normals) {
        glLineWidth(2.5);
        glColor3f(1.0, 0.0, 0.0);
        // glBegin(GL_LINES);
        // glVertex3f(0, 0, 0);
        // node_p.data()[0]+=tet_vertice_normal.data()[i*3+0];
        // node_p.data()[1]+=tet_vertice_normal.data()[i*3+1];
        // node_p.data()[2]+=tet_vertice_normal.data()[i*3+2];
        // glVertex3f(0,1.0,0);
        // glEnd();

        int num_sur_vet = tet_sur_vertice_normal.size() / 3;

        for (int i = 0; i < num_sur_vet; i++) {
            Eigen::Vector3d node_p;
            for(int j=0;j<3;j++)
            {
                node_p.data()[j] = tet_sur_vertice_col.data()[j*tet_sur_vertice_col.rows()+i];
            }

            glBegin(GL_LINES);
            glVertex3d(node_p.data()[0], node_p.data()[1], node_p.data()[2]);
            //node_p.data()[1] += 0.1;
             node_p.data()[0]+=tet_sur_vertice_normal.data()[0*tet_sur_vertice_normal.rows()+i]*0.2;
             node_p.data()[1]+=tet_sur_vertice_normal.data()[1*tet_sur_vertice_normal.rows()+i]*0.2;
             node_p.data()[2]+=tet_sur_vertice_normal.data()[2*tet_sur_vertice_normal.rows()+i]*0.2;
            glVertex3f(node_p.data()[0], node_p.data()[1], node_p.data()[2]);
            glEnd();
        }
    }
}

void Lobo::LoboTetMesh::initialGL() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    // glBindVertexArray(VAO);
    // glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    initializedGL = true;
    status_flags |= TetMeshStatusFlags_initialGL;
}

void Lobo::LoboTetMesh::reinitialTetMesh() {
    vertices_flags.resize(tet_vertice.size() / 3);
    std::fill(vertices_flags.begin(), vertices_flags.end(), 0);

    tet_vertice_col =
        Lobo::eigen_vec_2_mat(tet_vertice, tet_vertice.size() / 3, 3);
    tet_faces_col = Lobo::eigen_vec_2_mat(tet_faces, tet_faces.size() / 3, 3);
    tet_indices_col =
        Lobo::eigen_vec_2_mat(tet_indices, tet_indices.size() / 4, 4);

    tet_vertice_attri.resize(tet_vertice.size() / 3 * 11);
    tet_vertice_attri.setZero();

    tet_vertice_force.resize(tet_vertice.size());
    tet_vertice_force.setZero();

    setTetAttriColor(0.8, 0.8, 0.8);
    tet_faces_glint.resize(tet_faces.size());
    for (int i = 0; i < tet_faces.size(); i++) {
        tet_faces_glint[i] = tet_faces[i];
    }
    updateTetAttri(tet_vertice, 0, 3, 11);

    // default material
    setAllMaterial(1.0, 1000.0, 0.4);

    ori_tet_vertice = tet_vertice;

    status_flags &= ~TetMeshStatusFlags_precomputed;

    precomputeElementData();
}

void Lobo::LoboTetMesh::updateGL() {
    if (!(status_flags & TetMeshStatusFlags_datasizeUpdated)) {
        // no need updateGL
        return;
    }

    // update GL

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * tet_vertice_attri.size(),
                 tet_vertice_attri.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(unsigned int) * tet_faces_glint.size(),
                 tet_faces_glint.data(), GL_STATIC_DRAW);

    setPositionAttribute(0, 3, 11, 0);
    setPositionAttribute(1, 3, 11, 3);
    setPositionAttribute(2, 2, 11, 6);
    setPositionAttribute(3, 3, 11, 8);
    status_flags &= ~TetMeshStatusFlags_datasizeUpdated;
}

void Lobo::LoboTetMesh::updateTetVertices(Eigen::VectorXd *u) {
    tet_vertice = *u + ori_tet_vertice;
    tet_vertice_col =
        Lobo::eigen_vec_2_mat(tet_vertice, tet_vertice.size() / 3, 3);
    updateTetAttri(tet_vertice, 0, 3, 11);
    
    for(int i=0;i<surface_vertice_map_inverse.size();i++)
    {
        for(int k=0;k<3;k++)
        {
            int origin_index = surface_vertice_map_inverse[i];
            tet_sur_vertice_col.data()[k*tet_sur_vertice_col.rows()+i]
            = tet_vertice_col.data()[k*tet_vertice_col.rows()+origin_index];
        }
    }
    igl::per_vertex_normals(tet_sur_vertice_col, tet_sur_faces_col,
    tet_sur_vertice_normal);

    // update tri mesh
    if (lobomesh_binding != NULL) {
        int numtrinode = tri_ele_idl.size();
        Eigen::VectorXd buffer(numtrinode * 3);
        buffer.setZero();
        for (int i = 0; i < numtrinode; i++) {
            int eleid = tri_ele_idl[i];
            for (int j = 0; j < 4; j++) {
                int nodeid = tet_indices.data()[eleid * 4 + j];

                for (int k = 0; k < 3; k++) {
                    buffer.data()[i * 3 + k] +=
                        tet_vertice.data()[nodeid * 3 + k] *
                        tri_ele_weights.data()[i * 4 + j];
                }
            }
        }
        lobomesh_binding->updateVertices(buffer.data());
    }
}

void Lobo::LoboTetMesh::deleteGL() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void Lobo::LoboTetMesh::generateTet(const char *tetgen_command) {
    if (lobomesh_binding != NULL) {
        this->setInputPolygon(lobomesh_binding);
    }

    std::string command_ = "pq1.414Y";
    if (tetgen_command != NULL) {
        command_ = tetgen_command;
    }
    Eigen::MatrixXd TV;
    Eigen::MatrixXi TT;

    std::cout << "tetgen check" << std::endl;
    std::cout << "tri_vertices" << tri_vertices.rows() << " "
              << tri_vertices.cols() << std::endl;
    std::cout << "tri_faces" << tri_faces.rows() << " " << tri_faces.cols()
              << std::endl;

    int result = igl::copyleft::tetgen::tetrahedralize(
        tri_vertices, tri_faces, command_.c_str(), TV, TT, tet_faces_col);

    // test
    // Lobo::exportSimpleObj("test.obj",TV,TF);

    // copy data

    if (result == 0) {
        tet_vertice.resize(TV.rows() * TV.cols());
        for (int i = 0; i < TV.rows(); i++) {
            for (int j = 0; j < TV.cols(); j++) {
                tet_vertice.data()[i * TV.cols() + j] =
                    TV.data()[j * TV.rows() + i];
            }
        }
        tet_indices.resize(TT.rows() * TT.cols());
        for (int i = 0; i < TT.rows(); i++) {
            for (int j = 0; j < TT.cols(); j++) {
                // one tet has 4 vertices
                tet_indices.data()[i * TT.cols() + j] =
                    TT.data()[j * TT.rows() + i];
            }
        }

        tet_faces.resize(tet_faces_col.rows() * tet_faces_col.cols());
        for (int i = 0; i < tet_faces_col.rows(); i++) {
            for (int j = 0; j < tet_faces_col.cols(); j++) {
                tet_faces.data()[i * tet_faces_col.cols() + j] =
                    tet_faces_col.data()[j * tet_faces_col.rows() + i];
            }
        }

        status_flags |= TetMeshStatusFlags_datasizeUpdated;
        status_flags |= TetMeshStatusFlags_tetgened;
    } else {
        std::cout << "tetgen failed" << command_ << filebase << std::endl;
    }

    reinitialTetMesh();
}

void Lobo::LoboTetMesh::setBindingTriMesh(LoboMesh *lobomesh) {
    this->lobomesh_binding = lobomesh;
}

void Lobo::LoboTetMesh::setInputPolygon(LoboMesh *lobomesh) {
    // set vertices
    int num_tri_vertices = lobomesh->attrib.vertices.size() / 3;
    tri_vertices.resize(num_tri_vertices, 3);

    for (int i = 0; i < num_tri_vertices; i++) {
        tri_vertices.data()[i] = lobomesh->attrib.vertices[i * 3 + 0];
        tri_vertices.data()[i + num_tri_vertices] =
            lobomesh->attrib.vertices[i * 3 + 1];
        tri_vertices.data()[i + 2 * num_tri_vertices] =
            lobomesh->attrib.vertices[i * 3 + 2];
    }
    int num_tri_faces = lobomesh->num_faces;
    tri_faces.resize(num_tri_faces, 3);
    int face_index_slid = 0;
    for (int i = 0; i < lobomesh->shapes.size(); i++) {
        for (int j = 0; j < lobomesh->shapes[i].mesh.indices.size() / 3; j++) {
            tri_faces.data()[face_index_slid] =
                lobomesh->shapes[i].mesh.indices[j * 3].vertex_index;
            tri_faces.data()[face_index_slid + num_tri_faces] =
                lobomesh->shapes[i].mesh.indices[j * 3 + 1].vertex_index;
            tri_faces.data()[face_index_slid + 2 * num_tri_faces] =
                lobomesh->shapes[i].mesh.indices[j * 3 + 2].vertex_index;
            face_index_slid += 1;
        }
    }

    // test obj
    // Lobo::exportSimpleObj("test.obj",tri_vertices,tri_faces);
}

void Lobo::LoboTetMesh::setInputPolygon(Eigen::VectorXd *vertices,
                                        Eigen::VectorXi *faces) {
    tri_vertices.resize(vertices->rows(), vertices->cols());
    memcpy(tri_vertices.data(), vertices->data(),
           sizeof(double) * vertices->rows() * vertices->cols());
    tri_faces.resize(faces->rows(), faces->cols());
    memcpy(tri_faces.data(), faces->data(),
           sizeof(int) * faces->rows() * faces->cols());
}

void Lobo::LoboTetMesh::exportTetMesh() {
    if (usebinary) {
        exportTetMeshBinary(Lobo::getPath(filebase.c_str()).c_str());
    } else {
        exportTetMeshAscii(Lobo::getPath(filebase.c_str()).c_str());
    }
}
void Lobo::LoboTetMesh::loadTetMesh() {
    if (usebinary) {
        loadTetMeshBinary(Lobo::getPath(filebase.c_str()).c_str());
    } else {
        loadTetMeshAscii(Lobo::getPath(filebase.c_str()).c_str());
    }
}

void Lobo::LoboTetMesh::loadTetMeshBinary(const char *filebase_) {
    std::ostringstream stringStream;
    stringStream << filebase_ << ".tet";
    std::string filename = stringStream.str();

    std::cout << "loadTetMeshBinary " << filename << std::endl;

    std::ifstream in(filename, std::ios::in | std::ios::binary);
    if (!in.good()) {
        std::cout << filename << "file not open" << std::endl;
        return;
    }
    EigenMatrixIO::read_binary(in, tet_vertice);
    EigenMatrixIO::read_binary(in, tet_indices);
    EigenMatrixIO::read_binary(in, tet_faces);
    in.close();
    status_flags |= TetMeshStatusFlags_loadtet;
    status_flags |= TetMeshStatusFlags_datasizeUpdated;
    reinitialTetMesh();
}

void Lobo::LoboTetMesh::exportTetMeshBinary(const char *filebase_) {
    if (!(status_flags &
          (TetMeshStatusFlags_tetgened | TetMeshStatusFlags_loadtet))) {
        return;
    }
    std::ostringstream stringStream;
    stringStream << filebase_ << ".tet";
    std::string filename = stringStream.str();

    std::cout << "exportTetMeshBinary " << filename << std::endl;

    std::ofstream out(filename,
                      std::ios::out | std::ios::binary | std::ios::trunc);
    EigenMatrixIO::write_binary(out, tet_vertice);
    EigenMatrixIO::write_binary(out, tet_indices);
    EigenMatrixIO::write_binary(out, tet_faces);
    out.close();
}

void Lobo::LoboTetMesh::loadTetMeshAscii(const char *filebase_) {
    std::cout << "loadTetMeshAscii " << filebase_ << std::endl;

    std::ostringstream stringStream;
    stringStream << filebase_ << ".ele";
    std::string elementfile = stringStream.str();
    stringStream.str("");
    stringStream.clear();
    stringStream << filebase_ << ".node";
    std::string nodefile = stringStream.str();
    stringStream.str("");
    stringStream.clear();
    stringStream << filebase_ << ".face";
    std::string facefile = stringStream.str();

    int tmp;
    int numele, numvet, numface;
    std::ifstream inputstream(elementfile);
    inputstream >> numele >> tmp >> tmp;
    tet_indices.resize(numele * 4);
    for (int i = 0; i < numele; i++) {
        inputstream >> tmp;
        for (int j = 0; j < 4; j++) {
            inputstream >> tet_indices.data()[i * 4 + j];
        }
    }
    inputstream.close();

    inputstream.open(nodefile);
    inputstream >> numvet >> tmp >> tmp >> tmp;
    tet_vertice.resize(numvet * 3);
    for (int i = 0; i < numvet; i++) {
        inputstream >> tmp;
        for (int j = 0; j < 3; j++) {
            inputstream >> tet_vertice.data()[i * 3 + j];
        }
    }
    inputstream.close();

    inputstream.open(facefile);
    inputstream >> numface;
    tet_faces.resize(numface * 3);
    for (int i = 0; i < numface; i++) {
        inputstream >> tmp;
        for (int j = 0; j < 3; j++) {
            inputstream >> tet_faces.data()[i * 3 + j];
        }
    }
    inputstream.close();
    status_flags |= TetMeshStatusFlags_datasizeUpdated;
    status_flags |= TetMeshStatusFlags_loadtet;
    reinitialTetMesh();
}
void Lobo::LoboTetMesh::exportTetMeshAscii(const char *filebase_) {
    if (!(status_flags &
          (TetMeshStatusFlags_tetgened | TetMeshStatusFlags_loadtet))) {
        return;
    }
    std::cout << "exportTetMeshAscii " << filebase_ << std::endl;
    std::ostringstream stringStream;
    stringStream << filebase_ << ".ele";
    std::string elementfile = stringStream.str();
    stringStream.str("");
    stringStream.clear();
    stringStream << filebase_ << ".node";
    std::string nodefile = stringStream.str();
    stringStream.str("");
    stringStream.clear();
    stringStream << filebase_ << ".face";
    std::string facefile = stringStream.str();

    std::ofstream outstream(elementfile);
    outstream << tet_indices.size() / 4 << " 4 0 " << std::endl;
    for (int i = 0; i < tet_indices.rows() / 4; i++) {
        outstream << i << " ";
        for (int j = 0; j < 4; j++) {
            outstream << tet_indices.data()[i * 4 + j] << " ";
        }
        outstream << std::endl;
    }
    outstream.close();

    outstream.open(nodefile);
    outstream << tet_vertice.size() / 3 << " 3 0 0 " << std::endl;
    for (int i = 0; i < tet_vertice.rows() / 3; i++) {
        outstream << i << " ";
        for (int j = 0; j < 3; j++) {
            outstream << tet_vertice.data()[i * 3 + j] << " ";
        }
        outstream << std::endl;
    }
    outstream.close();
    outstream.open(facefile);
    outstream << tet_faces.size() / 3 << std::endl;
    for (int i = 0; i < tet_faces.rows() / 3; i++) {
        outstream << i << " ";
        for (int j = 0; j < 3; j++) {
            outstream << tet_faces.data()[i * 3 + j] << " ";
        }
        outstream << std::endl;
    }
    outstream.close();
}

void Lobo::LoboTetMesh::exportConstrainedVertices(const char *filename) {
    std::vector<unsigned int> constrained_DoFs;
    for (int i = 0; i < vertices_flags.size(); i++) {
        if (vertices_flags[i] == 1) {
            constrained_DoFs.push_back(i * 3);
            constrained_DoFs.push_back(i * 3 + 1);
            constrained_DoFs.push_back(i * 3 + 2);
        }
    }
    std::ofstream output(filename);
    output << constrained_DoFs.size() << std::endl;
    for (int i = 0; i < constrained_DoFs.size(); i++) {
        output << constrained_DoFs[i] << std::endl;
    }
    output.close();
}

void Lobo::LoboTetMesh::updateTetAttri(Eigen::VectorXd &inputattri, int offset,
                                       int attrisize, int totalsize) {
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < inputattri.size() / attrisize; i++) {
            for (int j = 0; j < attrisize; j++)
                tet_vertice_attri.data()[i * totalsize + offset + j] =
                    inputattri.data()[i * attrisize + j];
        }
    }
}

void Lobo::LoboTetMesh::updateTetAttri(const double *inputattri, int size,
                                       int offset, int attrisize,
                                       int totalsize) {
    for (int i = 0; i < size / attrisize; i++) {
        for (int j = 0; j < attrisize; j++)
            tet_vertice_attri.data()[i * totalsize + offset + j] =
                inputattri[i * attrisize + j];
    }
}

void Lobo::LoboTetMesh::setTetAttriColor(double r, double g, double b,
                                         int offset, int totalsize) {
    for (int i = 0; i < tet_vertice_attri.size() / totalsize; i++) {
        tet_vertice_attri.data()[i * totalsize + offset + 0] = r;
        tet_vertice_attri.data()[i * totalsize + offset + 1] = g;
        tet_vertice_attri.data()[i * totalsize + offset + 2] = b;
    }
}

void Lobo::LoboTetMesh::setTetVetAttriColor(int vid, double r, double g,
                                            double b, int offset,
                                            int totalsize) {
    tet_vertice_attri.data()[vid * totalsize + offset + 0] = r;
    tet_vertice_attri.data()[vid * totalsize + offset + 1] = g;
    tet_vertice_attri.data()[vid * totalsize + offset + 2] = b;
}

void Lobo::LoboTetMesh::setAllMaterial(double density, double youngsmodulus,
                                       double possionratio) {
    materials.clear();
    materialid.resize(getNumElements());
    std::fill(materialid.begin(), materialid.end(), 0);

    Material m;
    m.density = density;
    m.youngsmodulus = youngsmodulus;
    m.possionratio = possionratio;
    materials.push_back(m);
}

void Lobo::LoboTetMesh::computeDiagMassMatrix(
    Eigen::SparseMatrix<double> *mass) {
    int r = getNumVertex() * 3;
    mass->resize(r, r);
    typedef Eigen::Triplet<double> EIGEN_TRI_T;
    std::vector<EIGEN_TRI_T> coefficients;
    int numElements = getNumElements();
    for (int i = 0; i < numElements; i++) {
        double vol = elements_data[i].volume;
        double density = materials[materialid[i]].density;
        for (int j = 0; j < 4; j++) {
            double mass_value = 0;
            mass_value = density * vol / 4.0;
            int row = tet_indices[i * 4 + j] * 3;
            for (int d = 0; d < 3; d++) {
                coefficients.push_back(
                    EIGEN_TRI_T(row + d, row + d, mass_value));
            }
        }
    }
    mass->setFromTriplets(coefficients.begin(), coefficients.end());
}

void Lobo::LoboTetMesh::precomputeNodeData() {
    if (status_flags & TetMeshStatusFlags_precomputed) {
        // already precomputed
        return;
    }
    int numNodes = getNumVertex();
    nodes_data.resize(numNodes);
    searchNeighborNodes();
    constructSurfaceMesh();

    // tet_vertice_normal.resize(numNodes, 3);
    // igl::per_vertex_normals(tet_vertice_col, tet_faces_col,
    // tet_vertice_normal);
}

void Lobo::LoboTetMesh::precomputeElementData() {
    if (status_flags & TetMeshStatusFlags_precomputed) {
        // already precomputed
        return;
    }
    int numElements = getNumElements();
    elements_data.resize(numElements);

    mesh_total_volume = 0.0;
    for (int i = 0; i < numElements; i++) {
        correctElementNodeOrder(i);
        computeElementVolume(i);
        computeElementShapeFunctionDerivate(i);  // precompute dF_du
        mesh_total_volume += elements_data[i].volume;

        // update element center
        elements_data[i].center_p.setZero();
        for (int j = 0; j < 4; j++) {
            int nodeid = tet_indices.data()[i * 4 + j];
            elements_data[i].center_p.data()[0] +=
                tet_vertice.data()[nodeid * 3 + 0];
            elements_data[i].center_p.data()[1] +=
                tet_vertice.data()[nodeid * 3 + 1];
            elements_data[i].center_p.data()[2] +=
                tet_vertice.data()[nodeid * 3 + 2];
        }
        elements_data[i].center_p /= 4;
    }

    generateBarycentricCoordinate();

    precomputeNodeData();

    status_flags |= TetMeshStatusFlags_precomputed;
}

void Lobo::LoboTetMesh::getNodeElements(
    std::vector<std::vector<int>> &node_elements) {
    node_elements.resize(getNumVertex());
    int num_elements = getNumElements();
    for (int i = 0; i < num_elements; i++) {
        for (int j = 0; j < numElementVertices; j++) {
            int nodeid = tet_indices.data()[i * 4 + j];
            node_elements[nodeid].push_back(i);
        }
    }
}

void Lobo::LoboTetMesh::getNodeRestPosition(int nodeid, Eigen::Vector3d &p) {
    for (int j = 0; j < 3; j++) {
        p.data()[j] = ori_tet_vertice.data()[nodeid * 3 + j];
    }
}

Eigen::Vector3d Lobo::LoboTetMesh::getNodeRestPosition(int nodeid) {
    Eigen::Vector3d tmp;
    getNodeRestPosition(nodeid, tmp);
    return tmp;
}

Eigen::Vector3d Lobo::LoboTetMesh::getNodeCurPosition(int nodeid)
{
    Eigen::Vector3d tmp;
    tmp.setZero();
    for (int j = 0; j < 3; j++) {
        tmp.data()[j] = tet_vertice.data()[nodeid * 3 + j];
    }
    return tmp;
}

Eigen::Vector3d Lobo::LoboTetMesh::getNodeNormal(int nodeid)
{
    Eigen::Vector3d tmp;
    tmp.setZero();
    if(surface_vertce_map[nodeid] == -1)
    {
        return tmp;
    }
    int sur_node_id = surface_vertce_map[nodeid];

    tmp.data()[0] = tet_sur_vertice_normal.data()[tet_sur_vertice_normal.rows()*0+sur_node_id];
    tmp.data()[1] = tet_sur_vertice_normal.data()[tet_sur_vertice_normal.rows()*1+sur_node_id];
    tmp.data()[2] = tet_sur_vertice_normal.data()[tet_sur_vertice_normal.rows()*2+sur_node_id];

    return tmp;
}

void Lobo::LoboTetMesh::correctElementNodeOrder(int elementid) {
    int ni[4];
    Eigen::Vector3d node_p[4];
    for (int i = 0; i < 4; i++) {
        ni[i] = tet_indices[elementid * 4 + i];
        for (int j = 0; j < 3; j++) {
            getNodeRestPosition(ni[i], node_p[i]);
        }
    }
    Eigen::Vector3d direction_v;
    Lobo::computeTriangleNorm(node_p[0], node_p[1], node_p[2], direction_v);
    Eigen::Vector3d n3n0 = node_p[3] - node_p[0];
    if (n3n0.dot(direction_v) < 0) {
        // element->node_indices[1] = n2;
        // element->node_indices[2] = n1;
        tet_indices[elementid * 4 + 1] = ni[2];
        tet_indices[elementid * 4 + 2] = ni[1];
        std::cout << "bad order element" << std::endl;
    }
}

void Lobo::LoboTetMesh::computeElementVolume(int elementid) {
    Eigen::Vector3d a =
        this->getNodeRestPosition(tet_indices[elementid * 4 + 0]);
    Eigen::Vector3d b =
        this->getNodeRestPosition(tet_indices[elementid * 4 + 1]);
    Eigen::Vector3d c =
        this->getNodeRestPosition(tet_indices[elementid * 4 + 2]);
    Eigen::Vector3d d =
        this->getNodeRestPosition(tet_indices[elementid * 4 + 3]);

    elements_data[elementid].volume = Lobo::computeTetVolumeABS(a, b, c, d);
}

void Lobo::LoboTetMesh::computeElementShapeFunctionDerivate(int elementid) {
    int ni[4];
    Eigen::Vector3d node_p[4];
    for (int i = 0; i < 4; i++) {
        ni[i] = tet_indices[elementid * 4 + i];
        for (int j = 0; j < 3; j++) {
            getNodeRestPosition(ni[i], node_p[i]);
        }
    }

    TetElementData *te = &elements_data[elementid];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            te->Dm.data()[j * 3 + i] =
                node_p[j].data()[i] - node_p[3].data()[i];
        }
    }
    te->Dm_inverse = te->Dm.inverse();

    Eigen::Matrix4d referenceShape;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            referenceShape.data()[i * 4 + j] = node_p[i].data()[j];
        }
        referenceShape.data()[i * 4 + 3] = 1;
    }

    // need test
    Eigen::Matrix4d inverseShapefunction = referenceShape.inverse();
    te->shape_function_inv = inverseShapefunction;

    te->Phi_derivate.resize(4, 3);
    for (int i = 0; i < 4; i++) {
        te->Phi_derivate.data()[0 * 4 + i] =
            inverseShapefunction.data()[0 * 4 + i];
        te->Phi_derivate.data()[1 * 4 + i] =
            inverseShapefunction.data()[1 * 4 + i];
        te->Phi_derivate.data()[2 * 4 + i] =
            inverseShapefunction.data()[2 * 4 + i];
    }
}

void Lobo::LoboTetMesh::generateBarycentricCoordinate() {
    this->lobomesh_binding;
    int num_trinode = this->lobomesh_binding->attrib.vertices.size() / 3;

    tri_ele_weights.resize(num_trinode * 4);
    tri_vertices_idl.resize(num_trinode);

    // buil aabb tree for elemenet centers
    igl::AABB<Eigen::MatrixXd, 3> tree;
    tree.init(tet_vertice_col, tet_indices_col);

    Eigen::MatrixXd Q_trivertices(num_trinode, 3);

    for (int i = 0; i < num_trinode; i++) {
        for (int j = 0; j < 3; j++) {
            Q_trivertices.data()[j * num_trinode + i] =
                lobomesh_binding->attrib.vertices[i * 3 + j];
        }
    }

    Eigen::VectorXd sqrD;
    Eigen::MatrixXd C;
    // std::cout<<tet_vertice_col.cols()<< " " <<
    // tet_indices_col.cols()<<std::endl;
    Eigen::MatrixXi test_tet_faces(tet_indices_col.rows(), 3);
    test_tet_faces = tet_indices_col.block(0, 0, tet_indices_col.rows(), 3);

    tree.squared_distance(tet_vertice_col, test_tet_faces, Q_trivertices, sqrD,
                          tri_ele_idl, C);

    // igl::in_element(tet_vertice_col, tet_indices_col, Q_trivertices, tree,
    //                 tri_ele_idl);

    for (int i = 0; i < num_trinode; i++) {
        Eigen::Vector4d weights_;
        Eigen::Vector3d position_;
        position_.data()[0] = Q_trivertices.data()[0 * num_trinode + i];
        position_.data()[1] = Q_trivertices.data()[1 * num_trinode + i];
        position_.data()[2] = Q_trivertices.data()[2 * num_trinode + i];
        // if(tri_ele_idl.data()[i]==-1)
        // {
        //     tri_ele_idl.data()[i] = getCloesetElement(position_);
        //     std::cout<<"tri_ele_idl " << i << std::endl;
        // }

        computeBarycentricWeights(tri_ele_idl.data()[i], position_, weights_);

        tri_ele_weights.data()[i * 4 + 0] = weights_.data()[0];
        tri_ele_weights.data()[i * 4 + 1] = weights_.data()[1];
        tri_ele_weights.data()[i * 4 + 2] = weights_.data()[2];
        tri_ele_weights.data()[i * 4 + 3] = weights_.data()[3];

        // compute tri_vertices_idl
        double min_distance = DBL_MAX;
        int closest_index = -1;
        for (int j = 0; j < 4; j++) {
            int nodeid = tet_indices.data()[tri_ele_idl.data()[i] * 4 + j];
            Eigen::Vector3d ele_node = this->getNodeRestPosition(nodeid);
            double distance = (position_ - ele_node).norm();
            if (distance < min_distance) {
                closest_index = nodeid;
                min_distance = distance;
            }
        }
        tri_vertices_idl[i] = closest_index;
    }

    std::cout << "generateBarycentricCoordinate" << std::endl;
}

void Lobo::LoboTetMesh::computeBarycentricWeights(int eleid,
                                                  Eigen::Vector3d &pos,
                                                  Eigen::Vector4d &weights) {
    Eigen::Vector4d point;
    point.data()[0] = pos.data()[0];
    point.data()[1] = pos.data()[1];
    point.data()[2] = pos.data()[2];
    point.data()[3] = 1;

    if (eleid == -1) {
        weights.setZero();
        return;
    }

    weights = elements_data[eleid].shape_function_inv * point;
}

int Lobo::LoboTetMesh::getContainedElement(Eigen::Vector3d &position) {
    int numElements = this->getNumElements();
    for (int element = 0; element < numElements; element++) {
        if (containsVertex(element, position)) return element;
    }
    return -1;
}

int Lobo::LoboTetMesh::getCloesetElement(Eigen::Vector3d &position) {
    double distance = DBL_MAX;
    int eleid = -1;
    int numElements = this->getNumElements();
    for (int element = 0; element < numElements; element++) {
        double dis = (position - elements_data[element].center_p).norm();
        if (dis < distance) {
            eleid = element;
            distance = dis;
        }
    }
    return eleid;
}

bool Lobo::LoboTetMesh::containsVertex(int eleid, Eigen::Vector3d &pos) {
    Eigen::Vector4d weight;
    this->computeBarycentricWeights(eleid, pos, weight);

    return ((weight.data()[0] >= -1e-2) && (weight.data()[1] >= -1e-2) &&
            (weight.data()[2] >= -1e-2) && (weight.data()[3] >= -1e-2));
}

void Lobo::LoboTetMesh::searchNeighborNodes() {
    for (int i = 0; i < nodes_data.size(); i++) {
        nodes_data[i].neighbor.clear();
    }

    int numElements = getNumElements();
    for (int i = 0; i < numElements; i++) {
        for (int j = 0; j < 4; j++) {
            int nodeidj = tet_indices.data()[i * 4 + j];

            for (int m = j + 1; m < 4; m++) {
                int nodeidm = tet_indices.data()[i * 4 + m];
                if (!findElement(nodes_data[nodeidj].neighbor, nodeidm)) {
                    nodes_data[nodeidj].neighbor.push_back(nodeidm);
                }

                if (!findElement(nodes_data[nodeidm].neighbor, nodeidj)) {
                    nodes_data[nodeidm].neighbor.push_back(nodeidj);
                }
            }
        }
    }
}



void Lobo::LoboTetMesh::constructSurfaceMesh() {
    // search the
    Eigen::MatrixXd TT;
    // if there is no neighbor, the id will be -1

    igl::tet_tet_adjacency(tet_indices_col, TT);
    std::vector<bool> surface_vertce_flag(tet_vertice_col.rows());
    surface_vertce_map.resize(tet_vertice_col.rows());

    std::fill(surface_vertce_flag.begin(), surface_vertce_flag.end(), false);
    std::fill(surface_vertce_map.begin(), surface_vertce_map.end(), -1);

    std::vector<std::vector<int>> face_order(4);
    std::vector<Eigen::Vector3i> surface_faces;

    face_order[0].resize(4);
    face_order[0][0] = 0;
    face_order[0][1] = 1;
    face_order[0][2] = 2;
    face_order[0][3] = 3;


    face_order[1].resize(4);
    face_order[1][0] = 0;
    face_order[1][1] = 1;
    face_order[1][2] = 3;
    face_order[1][3] = 2;

    face_order[2].resize(4);
    face_order[2][0] = 1;
    face_order[2][1] = 2;
    face_order[2][2] = 3;
    face_order[2][3] = 0;

    face_order[3].resize(4);
    face_order[3][0] = 2;
    face_order[3][1] = 0;
    face_order[3][2] = 3;
    face_order[3][3] = 1;


    Eigen::Vector3i face_selected_tmp;
    for (int i = 0; i < TT.rows(); i++) {
        for (int j = 0; j < TT.cols(); j++) {
            if (TT.data()[j * TT.rows() + i] == -1) {
                
                for (int k = 0; k < 3; k++) {
                    int vertex_order = face_order[j][k];
                    int vertex_index =
                        tet_indices_col
                            .data()[vertex_order * tet_indices_col.rows() + i];
                    surface_vertce_flag[vertex_index] = true;


                    face_selected_tmp.data()[k] = vertex_index;
                }

                correct_face_order(i,face_order[j],face_selected_tmp);
                surface_faces.push_back(face_selected_tmp);
            }
        }
    }

    // reorganize the surface_faces
    int surface_vertex_count = 0;
    for (int i = 0; i < surface_vertce_flag.size(); i++) {
        if (surface_vertce_flag[i] == true) {
            surface_vertce_map[i] = surface_vertex_count;
            surface_vertex_count++;
        }
    }
    tet_sur_vertice_col.resize(surface_vertex_count, 3);
    tet_sur_faces_col.resize(surface_faces.size(), 3);
    tet_sur_vertice_normal.resize(surface_vertex_count, 3);
    surface_vertice_map_inverse.resize(surface_vertex_count,3);

    surface_vertex_count = 0;
    for (int i = 0; i < surface_vertce_flag.size(); i++) {
        if (surface_vertce_flag[i] == true) {
            surface_vertice_map_inverse[surface_vertex_count] = i;
            for (int j = 0; j < 3; j++) {
                tet_sur_vertice_col.data()[j * tet_sur_vertice_col.rows() +
                                           surface_vertex_count] =
                    tet_vertice_col.data()[j * tet_vertice_col.rows() + i];
            }
            surface_vertex_count++;
        }
    }

    for (int i = 0; i < surface_faces.size(); i++) {
        for (int j = 0; j < 3; j++) {
            int nodei = surface_faces[i].data()[j];
            int new_nodei = surface_vertce_map[nodei];
            tet_sur_faces_col.data()[j*tet_sur_faces_col.rows()+i] = new_nodei;
        }
    }


    igl::per_vertex_normals(tet_sur_vertice_col, tet_sur_faces_col,
    tet_sur_vertice_normal);



}

void Lobo::LoboTetMesh::correct_face_order(int eleid, std::vector<int> face_order,Eigen::Vector3i &face_index)
{
    int n[4];
    Eigen::Vector3d nodep[4];

    for(int i=0;i<4;i++)
    {
        
        n[i] = tet_indices_col.data()[face_order[i] * tet_indices_col.rows() + eleid];
        nodep[i] = this->getNodeRestPosition(n[i]);
    }

    //compute face_normal 
    Eigen::Vector3d direction_v;
    Lobo::computeTriangleNorm(nodep[0], nodep[1], nodep[2], direction_v);
    Eigen::Vector3d n3n0 = nodep[3] - nodep[0];
    if (n3n0.dot(direction_v) > 0) {
        // element->node_indices[1] = n2;
        // element->node_indices[2] = n1;
        face_index[1] = n[2];
        face_index[2] = n[1];
    }
}