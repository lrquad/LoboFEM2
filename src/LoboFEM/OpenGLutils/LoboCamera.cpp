#include "LoboCamera.h"

Lobo::Camera *current_camera = NULL;

void Lobo::setCurrentCamera(Camera *camera)
{
    current_camera = camera;
}

Lobo::Camera *Lobo::getCurrentCamera()
{
    return current_camera;
}

Lobo::Camera::Camera(glm::vec3 position,
                     glm::vec3 up, float yaw,
                     float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
                                    CenterP(glm::vec3(0.0f, 0.0f, 0.0f)),
                                    MovementSpeed(SPEED),
                                    MouseSensitivity(SENSITIVITY),
                                    Zoom(ZOOM),
                                    flymode(FLYMODE)
{
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;

    init_camera_value.Position = Position;
    init_camera_value.CenterP = CenterP;
    init_camera_value.Front = Front;
    init_camera_value.Yaw = Yaw;
    init_camera_value.Pitch = Pitch;

    updateCameraVectors();
}

Lobo::Camera::Camera(float posX, float posY, float posZ, float upX, float upY, float upZ,
                     float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
                                               MovementSpeed(SPEED),
                                               CenterP(glm::vec3(0.0f, 0.0f, 0.0f)),
                                               MouseSensitivity(SENSITIVITY),
                                               Zoom(ZOOM),
                                               flymode(FLYMODE)
{
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    Yaw = yaw;
    Pitch = pitch;
    init_camera_value.Position = Position;
    init_camera_value.CenterP = CenterP;
    init_camera_value.Front = Front;
    init_camera_value.Yaw = Yaw;
    init_camera_value.Pitch = Pitch;
    updateCameraVectors();
}

void Lobo::Camera::initCamera(glm::vec3 position,
                              glm::vec3 up, float yaw,
                              float pitch)
{
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;

    init_camera_value.Position = Position;
    init_camera_value.CenterP = CenterP;
    init_camera_value.Front = Front;
    init_camera_value.Yaw = Yaw;
    init_camera_value.Pitch = Pitch;

    updateCameraVectors();
}

void Lobo::Camera::drawImGui(bool *p_open)
{
    ImGui::Begin("Camera control", p_open,
                 ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Camera position: %.3f %.3f %.3f ", Position.x, Position.y,
                Position.z);
    ImGui::Text("Rotation center: %.3f %.3f %.3f ", CenterP.x, CenterP.y,
                CenterP.z);
    ImGui::Text("Yaw: %.3f Pitch: %.3f", Yaw, Pitch);
    ImGui::Checkbox("Flymode", &flymode);
    ImGui::InputFloat("MovementSpeed", &MovementSpeed, 0.1f, 1.0f, "%.3f");
    ImGui::InputFloat("MouseSensitivity", &MouseSensitivity, 0.01f, 1.0f,
                      "%.3f");
    if (ImGui::Button("Reset center"))
    {
        CenterP = glm::vec3(0.0);
        updateCameraVectors();
    };
    ImGui::SameLine();
    if (ImGui::Button("Reset Camera"))
    {
        Position = init_camera_value.Position;
        CenterP = init_camera_value.CenterP;
        Front = init_camera_value.Front;
        Yaw = init_camera_value.Yaw;
        Pitch = init_camera_value.Pitch;
        updateCameraVectors();
    };
    if (ImGui::Button("Save Camera"))
    {
        CSimpleIniA ini;
        char buffer[50];
        ini.SetUnicode();
        Lobo::setValueVec3(&ini, "Camera", "Position", Position);
        Lobo::setValueVec3(&ini, "Camera", "CenterP", CenterP);
        Lobo::setValuef(&ini, "Camera", "Yaw", Yaw);
        Lobo::setValuef(&ini, "Camera", "Pitch", Pitch);
        ini.SaveFile("./config/camera.ini");
    };
    ImGui::SameLine();
    if (ImGui::Button("Load Camera"))
    {
        loadCamera();
    };

    // ImGui::ColorEdit3("clear color", (float*)&clear_color);
    ImGui::End();

    flythroughMoveMode();
}

void Lobo::Camera::loadCamera()
{
    CSimpleIniA ini;
    ini.SetUnicode();
    ini.LoadFile("./config/camera.ini");
    Lobo::getValueVec3(&ini, "Camera", "Position", &Position);
    Lobo::getValueVec3(&ini, "Camera", "CenterP", &CenterP);
    Lobo::getValuef(&ini, "Camera", "Yaw", &Yaw);
    Lobo::getValuef(&ini, "Camera", "Pitch", &Pitch);
    updateCameraVectors();
}

void Lobo::Camera::flythroughMoveMode()
{
    ImGuiIO &io = ImGui::GetIO();

    float deltaTime = 1.0f / ImGui::GetIO().Framerate;

    if (!io.WantCaptureMouse)
    {
        if (flymode)
        {
            if (io.KeysDownDuration[int('W')] >= 0.0f)
            {
                ProcessKeyboard(FORWARD, deltaTime);
            }
            if (io.KeysDownDuration[int('A')] >= 0.0f)
            {
                ProcessKeyboard(LEFT, deltaTime);
            }
            if (io.KeysDownDuration[int('S')] >= 0.0f)
            {
                ProcessKeyboard(BACKWARD, deltaTime);
            }
            if (io.KeysDownDuration[int('D')] >= 0.0f)
            {
                ProcessKeyboard(RIGHT, deltaTime);
            }
            if (io.KeysDownDuration[341] >= 0.0f)
            {
                ProcessKeyboard(DOWNWARD, deltaTime);
            }
            if (io.KeysDownDuration[32] >= 0.0f)
            {
                ProcessKeyboard(UPWARD, deltaTime);
            }
        }

        if (io.MouseDownDuration[0] >= 0.0f)
        {
            ProcessMouseMovement(io.MouseDelta.x, io.MouseDelta.y);
        }

        if (io.MouseDownDuration[2] >= 0.0f)
        {
            ProcessMiddleMouseMovement(io.MouseDelta.x, io.MouseDelta.y);
        }

        ProcessMouseScroll(io.MouseWheel);
    }
}

glm::mat4 Lobo::Camera::GetViewMatrix()
{
    return glm::lookAt(Position, Position + Front, Up);
}

void Lobo::Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
    float velocity = MovementSpeed * deltaTime;
    if (direction == FORWARD)
    {
        Position += Front * velocity;
        CenterP += Front * velocity;
    }
    if (direction == BACKWARD)
    {
        Position -= Front * velocity;
        CenterP -= Front * velocity;
    }
    if (direction == LEFT)
    {
        Position -= Right * velocity;
        CenterP -= Right * velocity;
    }
    if (direction == RIGHT)
    {
        Position += Right * velocity;
        CenterP += Right * velocity;
    }
    if (direction == DOWNWARD)
    {
        Position -= Up * velocity;
        CenterP -= Up * velocity;
    }
    if (direction == UPWARD)
    {
        Position += Up * velocity;
        CenterP += Up * velocity;
    }
}

void Lobo::Camera::ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch)
{
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw += xoffset;
    Pitch -= yoffset;

    // Make sure that when pitch is out of bounds, screen doesn't get
    // flipped
    if (constrainPitch)
    {
        if (Pitch > 89.0f)
            Pitch = 89.0f;
        if (Pitch < -89.0f)
            Pitch = -89.0f;
    }

    // Update Front, Right and Up Vectors using the updated Euler angles
    updateCameraVectors();
}

void Lobo::Camera::ProcessMiddleMouseMovement(float xoffset, float yoffset)
{
    xoffset *= MouseSensitivity * 0.005;
    yoffset *= MouseSensitivity * 0.005;

    // Yaw += xoffset;
    // Pitch -= yoffset;
    Position += Up * yoffset;
    Position -= Right * xoffset;
    CenterP += Up * yoffset;
    CenterP -= Right * xoffset;
    // Update Front, Right and Up Vectors using the updated Euler angles
    updateCameraVectors();
}

void Lobo::Camera::ProcessMouseScroll(float yoffset)
{
    yoffset *= MouseSensitivity;
    Position += Front * yoffset;
    // CenterP += Front * yoffset;
}

void Lobo::Camera::updateCameraVectors()
{
    // Calculate the new Front vector
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));

    Front = glm::normalize(front);
    // Also re-calculate the Right and Up vector
    Right = glm::normalize(glm::cross(
        Front, WorldUp)); // Normalize the vectors, because their length
                          // gets closer to 0 the more you look up or down
                          // which results in slower movement.
    Up = glm::normalize(glm::cross(Right, Front));

    if (flymode == false)
    {
        float len = glm::length(Position - CenterP);
        Position = -Front * len + CenterP;
    }
}