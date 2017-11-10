#include "./viewer_camera.h"

#include <glm/gtc/matrix_transform.hpp>

void ViewerCamera::reshapeScreen(int width, int height) {
    this->width = width;
    this->height = height;
}

void ViewerCamera::move(float dx, float dy, float dz) {
    dx *= 0.05;  // scaling
    dy *= 0.05;
    dz *= 0.05;

    glm::vec3 dir_l = (this->to - this->from);
    glm::vec3 right_l = glm::cross(dir_l, this->up);
    glm::vec3 up_l = glm::cross(right_l, dir_l);

    dir_l *= dx / glm::length(dir_l);
    right_l *= dy / glm::length(right_l);
    up_l *= dz / glm::length(up_l);

    this->from += dir_l + right_l + up_l;
    this->to += right_l + up_l;
}

void ViewerCamera::rotateOrbit(float dtheta, float dphi) {
    dtheta *= 0.005;  // scaling
    dphi *= 0.005;

    glm::vec3 dir = this->to - this->from;
    float dir_norm = glm::length(dir);
    dir *= 1.0 / dir_norm;  // normalize

    float theta = atan2f(dir.x, dir.z);
    float phi = asinf(dir.y);
    theta += dtheta;
    phi += dphi;
    // Check phi range
    const float HALF_PIE = M_PI * 0.5f - std::numeric_limits<float>::epsilon();
    if (HALF_PIE < phi)
        phi = HALF_PIE;
    else if (phi < -HALF_PIE)
        phi = -HALF_PIE;

    dir.x = cosf(phi) * sinf(theta);
    dir.y = sinf(phi);
    dir.z = cosf(phi) * cosf(theta);

    dir *= dir_norm;  // reverce normalize
    this->from = this->to - dir;
}

glm::mat4 ViewerCamera::getProjectionMatrix(bool perspective) {
    // Perspective
    if (perspective) {
        float aspect = 1.f;
        if (width > 0 && height > 0) {
            aspect = static_cast<float>(width) / static_cast<float>(height);
        }
        return glm::perspective((fov / 180.f) * glm::pi<float>(), aspect,
                                0.001f, 10000.0f);
    } else {
        // Parallel
        float half_w = static_cast<float>(width) * 0.5f;
        float half_h = static_cast<float>(height) * 0.5f;
        return glm::ortho(-half_w, half_w, -half_h, half_h, 0.001f, 10000.0f);
    }
}

glm::mat4 ViewerCamera::getViewMatrix() { return glm::lookAt(from, to, up); }
