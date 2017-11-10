#ifndef GL_CAMERA_160412
#define GL_CAMERA_160412

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "render/gl_window.h"

class ViewerCamera : public GLCamera {
public:
    ViewerCamera() : from(0, 100, -100), to(0, 10, 20), up(0, 1, 0), fov(60) {}
    void reshapeScreen(int width, int height);
    void move(float dx, float dy, float dz);
    void rotateOrbit(float dtheta, float dphi);
    glm::mat4 getProjectionMatrix(bool perspective);
    glm::mat4 getViewMatrix();

private:
    int width, height;
    glm::vec3 from, to, up;
    float fov;
};

#endif
