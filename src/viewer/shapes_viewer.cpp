#include "shapes_viewer.h"

#include <iostream>
#include "render/gl_utils.h"

namespace {

void setCameraMatrix(ViewerCamera& camera, bool perspective) {
    glm::mat4 mv_mat = camera.getViewMatrix() * glm::mat4(1.0);
    glm::mat4 p_mat = camera.getProjectionMatrix(perspective);
    // model view
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixf(&mv_mat[0][0]);
    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixf(&p_mat[0][0]);
    checkGlError(101);
}

}  // namespace

bool ShapesViewer::init(const std::string& title, int vsync_interval) {
    std::cout << "* Create viewer windnow: " << title << std::endl;
    bool gl_ret = GLWindow::init(title, true, vsync_interval);
    if (!gl_ret) return false;
    GLWindow::setCamera(&camera);
    return true;
}

void ShapesViewer::update(bool perspective) {
    // set camera
    setCameraMatrix(camera, perspective);
    // execute opengl and poll events
    GLWindow::update();
}
