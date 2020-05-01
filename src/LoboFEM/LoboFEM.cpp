#include <iomanip>
#include "LoboFEM.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include "stb_image_write_o.h"

Lobo::LoboFEM::LoboFEM()
{
    dynamic_scene = NULL;
    scene = NULL;

    use_screen_buffer = false;
    export_screen_buffer = false;
    multisamples = 16;
    enablevsync = true;
}

Lobo::LoboFEM::~LoboFEM()
{
    delete dynamic_scene;
    delete scene;
}

void Lobo::LoboFEM::initScreenBuffer()
{
    float quadVertices[] = {// vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
                            // positions   // texCoords
                            -1.0f, 1.0f, 0.0f, 1.0f,
                            -1.0f, -1.0f, 0.0f, 0.0f,
                            1.0f, -1.0f, 1.0f, 0.0f,

                            -1.0f, 1.0f, 0.0f, 1.0f,
                            1.0f, -1.0f, 1.0f, 0.0f,
                            1.0f, 1.0f, 1.0f, 1.0f};
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));

    unsigned int SCR_WIDTH = 1280;
    unsigned int SCR_HEIGHT = 720;
    screenShader.useProgram();
    screenShader.setInt("screenTexture", 0);
    // framebuffer configuration
    // -------------------------
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // create a multisampled color attachment texture
    glGenTextures(1, &textureColorBufferMultiSampled);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples, GL_RGB, SCR_WIDTH, SCR_HEIGHT, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled, 0);
    // create a (also multisampled) renderbuffer object for depth and stencil attachments
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, multisamples, GL_DEPTH24_STENCIL8, SCR_WIDTH, SCR_HEIGHT);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // configure second post-processing framebuffer
    glGenFramebuffers(1, &intermediateFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediateFBO);
    // create a color attachment texture
    glGenTextures(1, &screenTexture);
    glBindTexture(GL_TEXTURE_2D, screenTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screenTexture, 0); // we only need a color buffer

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "ERROR::FRAMEBUFFER:: Intermediate framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Lobo::LoboFEM::windowLoop(GLFWwindow *window)
{
    //update scene before rendering
    scene->collision_world.doDetection();
    dynamic_scene->update();

    ImGui::NewFrame();
    drawImGui();
    ImGui::Render();

    //shadow rendering
    simpleDepthShader.useProgram();
    int numlights = light_manager.getLightNum();
    glEnable(GL_DEPTH_TEST);
    for (int i = 0; i < numlights; i++)
    {
        if (!light_manager.getLightTrigger(i) || !light_manager.getLightCastShadow(i))
        {
            continue;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, light_manager.getDepthFBO(i));
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        light_manager.getTextureSize(SHADOW_WIDTH, SHADOW_HEIGHT, i);
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glClear(GL_DEPTH_BUFFER_BIT);

        glm::mat4 shadowmodel = glm::mat4(1.0f);
        simpleDepthShader.setMat4("model", shadowmodel);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        light_manager.setLightShadow(&simpleDepthShader, i);

        scene->paintGL(&simpleDepthShader);
        dynamic_scene->paintGL(&default_shader, true);

        glCullFace(GL_BACK);
        glDisable(GL_CULL_FACE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    int display_w, display_h;
    // glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &display_w, &display_h);

    //
    //display_w*=0.8;

    if (use_screen_buffer)
    {
        //framebuffer_size_callback(window, display_w, display_h);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    }

    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z,
                 clear_color.w);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glDisable(GL_FRAMEBUFFER_SRGB);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // more opengl stuff
    // create transformations
    glm::mat4 model = glm::mat4(1.0f);

    glm::mat4 projection = glm::perspective(
        glm::radians(camera.Zoom), (float)display_w / (float)display_h,
        0.001f, 100.0f);
    glm::mat4 view = camera.GetViewMatrix();

    //set global context;
    camera.view_matrix = view;
    camera.projection_matrix = projection;
    camera.view_port[0] = 0.0;
    camera.view_port[1] = 0.0;
    camera.view_port[2] = display_w;
    camera.view_port[3] = display_h;

    default_shader.useProgram();
    default_shader.setMat4(
        "projection",
        projection); // note: currently we set the projection matrix each
                     // frame, but since the projection matrix rarely
                     // changes it's often best practice to set it outside
                     // the main loop only once.
    default_shader.setMat4("view", view);
    default_shader.setMat4("model", model);

    // default_shader.setVec3("lightPos",cubelight.lightPos);
    // default_shader.setVec3("lightColor",cubelight.lightColor);
    default_shader.setVec3("viewPos", camera.Position);
    light_manager.setLight(&default_shader);

    scene->paintGL(&default_shader);
    dynamic_scene->paintGL(&default_shader, false);

    lighting_shader.useProgram();
    lighting_shader.setMat4(
        "projection",
        projection); // note: currently we set the projection matrix each
                     // frame, but since the projection matrix rarely
                     // changes it's often best practice to set it outside
                     // the main loop only once.
    lighting_shader.setMat4("view", view);
    lighting_shader.setMat4("model", model);

    light_manager.paintGL(&lighting_shader);

    if (use_screen_buffer)
    {
        // 2. now blit multisampled buffer(s) to normal colorbuffer of intermediate FBO. Image is stored in screenTexture
        glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediateFBO);
        glBlitFramebuffer(0, 0, display_w, display_h, 0, 0, display_w, display_h, GL_COLOR_BUFFER_BIT, GL_NEAREST);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
        // clear all relevant buffers
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f); // set clear color to white (not really necessery actually, since we won't be able to see behind the quad anyways)
        glClear(GL_COLOR_BUFFER_BIT);
        screenShader.useProgram();
        screenShader.setInt("screenTexture", 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, screenTexture);
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }

    //make imgui SRGB
    glEnable(GL_FRAMEBUFFER_SRGB);

    if (export_screen_buffer)
    {
        saveCurScreenImagePNGAnimaition("./demo/default/screen/animation");
    }
}

void Lobo::LoboFEM::drawImGui()
{
    bool show_demo_window = false;

    showMainWindow(&fileDialog);

    light_manager.drawImGui();
    camera.drawImGui();
    scene->drawImGui();
    dynamic_scene->drawImGui();

    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);
}

void Lobo::LoboFEM::showMainWindow(ImGui::FileBrowser *fileDialog, bool *p_open)
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open", "Ctrl+O"))
            {
                fileDialog->Open();
                //ImGui::FileBrowser filebrowser;
                //filebrowser.Display();
            }
            if (ImGui::MenuItem("Save", "Ctrl+S"))
            {
            }
            ImGui::EndMenu();
        }

        ImGui::Text("| %.3f ms/frame (%.1f FPS) | %s |%d", 1000.0f * ImGui::GetIO().DeltaTime, 1.0 / ImGui::GetIO().DeltaTime, config_file_path.c_str(), dynamic_scene->getStep());

        ImGui::EndMainMenuBar();
    }

    fileDialog->Display();

    if (fileDialog->HasSelected())
    {
        //config_file_path = fileDialog->GetSelected().string();
        fileDialog->ClearSelected();
        this->loadXMLfile(config_file_path.c_str());
    }

    ImGuiIO &io = ImGui::GetIO();
    ImGui::Begin("Program setting",
                 p_open); // Create a window called "Hello, world!" and
                          // append into it.
    ImGui::Checkbox("OffScreenBuffer", &use_screen_buffer);
    if (ImGui::Button("Save Screen"))
    {
        this->saveCurScreenImagePNG("./demo/default/screen/screenshot.png");
    }
    ImGui::Checkbox("Export screen buffer", &export_screen_buffer);
    
    if (ImGui::Button("Generate Animation"))
    {
       export_screen_buffer = false;
       system("yes | ffmpeg -r 60 -f image2 -i ./demo/default/screen/animation%05d.png -vcodec libx264 -crf 10  -pix_fmt yuv420p ./demo/default/screen/screen_recored.mp4");
       system("yes | ffmpeg -i ./demo/default/screen/screen_recored.mp4 -vf fps=15,scale=640:-1:flags=lanczos,palettegen ./demo/default/screen/palette.png");
       system("yes | ffmpeg -i ./demo/default/screen/screen_recored.mp4 -i ./demo/default/screen/palette.png -filter_complex \"fps=15,scale=640:-1:flags=lanczos[x];[x][1:v]paletteuse\" ./demo/default/screen/screen_recored.gif");
    }

    if (ImGui::Button("Clear screen results"))
    {
       system("rm -rf ./demo/default/screen");
       system("mkdir -p ./demo/default/screen/");
    }

    ImGui::Checkbox("Enable vsync", &enablevsync);

    ImGui::End();
}

void Lobo::LoboFEM::paintGL(LoboShader *shader)
{
}

void Lobo::LoboFEM::deleteGL()
{
    dynamic_scene->deleteGL();
    scene->deleteGL();
}

void Lobo::LoboFEM::makeContext()
{
    clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    //scripts
    scene = new Lobo::LoboScene();
    //scene->addMesh("./models/floor.obj", false);
    //scene->addMesh("./models/earth/earth.obj", true);
    scene->initialGL();

    dynamic_scene = new Lobo::LoboDynamicScene(scene);
    // dynamic_scene->bindTetMesh(1, "tetmesh/bunny", false, true);
    dynamic_scene->initialGL();

    default_shader.loadShaderFile("./shaders/simplevertex.glsl",
                                  "./shaders/simplefrag.glsl");
    lighting_shader.loadShaderFile("./shaders/lightvertex.glsl",
                                   "./shaders/lightfrag.glsl");
    simpleDepthShader.loadShaderFile("./shaders/depthvertex.glsl",
                                     "./shaders/depthfrag.glsl");
    debugDepthQuad.loadShaderFile("./shaders/debugdepth.vs",
                                  "./shaders/debugdepth.fs");
    screenShader.loadShaderFile("./shaders/framebuffer_screen.vs",
                                "./shaders/framebuffer_screen.fs");
    camera.initCamera(glm::vec3(0.0, 3.0f, 8.0f), glm::vec3(0.0, 1.0, 0.0), -90, -16.0);
}
void Lobo::LoboFEM::setCurrentContext()
{
    //
    Lobo::setCurrentCamera(&camera);
}

void Lobo::LoboFEM::loadXMLfile(const char *filename)
{
    config_file_path = filename;
    deleteGL();
    delete dynamic_scene;
    delete scene;
    scene = new Lobo::LoboScene();
    dynamic_scene = new Lobo::LoboDynamicScene(scene);

    std::cout << "load scene script" << filename << std::endl;
    pugi::xml_parse_result result = xml_doc.load_file(filename);
    if (!result)
        std::cout << "xml load failed" << std::endl;

    if (xml_doc.child("Scene").child("LoboScene"))
    {
        pugi::xml_node scene_node = xml_doc.child("Scene").child("LoboScene");
        scene->runXMLscript(scene_node);
    }

    scene->initialGL();

    if (xml_doc.child("Scene").child("LoboDynamicScene"))
    {
        pugi::xml_node scene_node = xml_doc.child("Scene").child("LoboDynamicScene");
        dynamic_scene->runXMLscript(scene_node);
    }
    dynamic_scene->initialGL();

    std::cout << "Finished." << std::endl;
    setCurrentContext();
}

void Lobo::LoboFEM::framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples, GL_RGB, width, height, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, multisamples, GL_DEPTH24_STENCIL8, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, screenTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Lobo::LoboFEM::saveCurScreenImagePNG(const char *imagename)
{
    if (!this->use_screen_buffer)
    {
        std::cout << "no offline buffer" << std::endl;
        return;
    }
    glBindTexture(GL_TEXTURE_2D, screenTexture);

    int w = Lobo::getCurrentCamera()->view_port[2];
    int h = Lobo::getCurrentCamera()->view_port[3];

    float *read = (float *)malloc(sizeof(float) * w * h * 3);
    stbi_flip_vertically_on_write(true);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, read);
    // sizeof(float) * width * height * 3

    stbi_write_png(imagename, w, h, 3, read, 0);

    glBindTexture(GL_TEXTURE_2D, 0);
    free(read);
}

void Lobo::LoboFEM::saveCurScreenImagePNGAnimaition(const char *imagebase)
{
    if(this->dynamic_scene->getSaveStep()>2000)
    {
        return;
    }
    std::ostringstream stringStream;
    stringStream << imagebase << std::setfill('0') << std::setw(5) << this->dynamic_scene->getSaveStep() << ".png";
    std::string pngfile = stringStream.str();
    saveCurScreenImagePNG(pngfile.c_str());
}