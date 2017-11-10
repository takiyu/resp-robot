#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <stdio.h>
#include <string>

#include "../fps.h"
#include "../viewer/gl_shapes.h"
#include "../viewer/shapes_viewer.h"

const std::string MESH_TEST_PATH = "../3rdParty/mesh/face.obj";

int main(int argc, char const* argv[]) {
    // Shapes viewer
    ShapesViewer shapes_viewer(1024, 512);
    shapes_viewer.init("viewer", 1);

    // Test mesh
    GLMesh mesh;
    mesh.loadObjFile(MESH_TEST_PATH);

    // Test line
    GLLine line;
    line.setColor(glm::vec3(0, 1, 0));
    line.setHeadSize(10);
    glm::vec3 head(0), tail(0);

    // Fps
    FpsCounter fps;

    printf("* Rendering Loop\n");
    while (!shapes_viewer.shouldClose()) {
        // Setup window
        shapes_viewer.activate();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Change line status
        line.setHeadPosition(head);
        line.setTailPosition(tail);
        head[0] -= 0.01;

        // Draw
        mesh.draw();
        line.draw();

        // Update viewer
        shapes_viewer.update();

        std::cout << fps.update() << std::endl;
    }

    printf("* Exit\n");

    return 0;
}
