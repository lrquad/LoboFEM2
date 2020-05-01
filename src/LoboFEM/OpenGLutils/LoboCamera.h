#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include "Utils/simpleini/SimpleIni.h"
#include "Utils/simpleini/SimpleiniWarp.h"
#include "imgui.h"

// Defines several possible options for camera movement. Used as abstraction to
// stay away from window-system specific input methods
enum Camera_Movement
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UPWARD,
    DOWNWARD
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 2.5f;
const float SENSITIVITY = 0.5f;
const float ZOOM = 45.0f;
const bool FLYMODE = false;

namespace Lobo
{
class Camera;

void setCurrentCamera(Camera *camera);
Camera *getCurrentCamera();
// An abstract camera class that processes input and calculates the
// corresponding Euler Angles, Vectors and Matrices for use in OpenGL
struct InitCamera
{
    glm::vec3 Position;
    glm::vec3 CenterP;
    glm::vec3 Front;
    // Euler Angles
    float Yaw;
    float Pitch;
};

class Camera
{
public:
    // CSimpleIniA camera_ini_io;

    // Constructor with vectors
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
           glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW,
           float pitch = PITCH);

    // Constructor with scalar values
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ,
           float yaw, float pitch);

    void initCamera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
           glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW,
           float pitch = PITCH);

    void drawImGui(bool *p_open = NULL);

    void loadCamera();

    void flythroughMoveMode();

    // Returns the view matrix calculated using Euler Angles and the LookAt
    // Matrix
    glm::mat4 GetViewMatrix();

    // Processes input received from any keyboard-like input system. Accepts
    // input parameter in the form of camera defined ENUM (to abstract it from
    // windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);

    // Processes input received from a mouse input system. Expects the offset
    // value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true);

    void ProcessMiddleMouseMovement(float xoffset, float yoffset);

    // Processes input received from a mouse scroll-wheel event. Only requires
    // input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset);

    // Camera Attributes
    glm::vec3 Position;
    glm::vec3 CenterP;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    // Euler Angles
    float Yaw;
    float Pitch;
    // Camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    glm::vec3 angular_v;
    glm::vec3 angular_a;

    bool flymode;

    glm::vec2 mouse_position;

    InitCamera init_camera_value;


    glm::mat4 view_matrix;
    glm::mat4 projection_matrix;
    glm::vec4 view_port;

private:
    // Calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors();
};
} // namespace Lobo
#endif