#ifndef SHAPES_VIEWER_161024
#define SHAPES_VIEWER_161024

#include "render/gl_window.h"
#include "viewer_camera.h"

class ShapesViewer : public GLWindow {
public:
    ShapesViewer(int width = 500, int height = 500) : GLWindow(width, height) {}

    bool init(const std::string &title, int vsync_interval);
    void update(bool perspective = true);

private:
    ViewerCamera camera;
};

#endif
