#ifndef GLSHAPE_H_160518
#define GLSHAPE_H_160518
#include <string>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

const float LIGHT0_POSITION[] = {0, 0, -200, 1};

class GLBaseShape {
public:
    GLBaseShape()
        : visibility(true),
          color(0.8f),
          translate(0.f),
          scale(1.f),
          angle(0.f){};
    void setVisibility(bool v) { visibility = v; }
    void setColor(const glm::vec3& c) { color = c; }
    void setTranslation(const glm::vec3& t) { translate = t; }
    void setScale(const float s) { scale = s; }
    void setAngle(const glm::vec3& r) { angle = r; }
    void draw();

protected:
    bool visibility;
    glm::vec3 color;
    glm::vec3 translate;
    float scale;
    glm::vec3 angle;  // degree
    virtual void drawGL() = 0;
};

class GLMesh : public GLBaseShape {
public:
    GLMesh() {}
    ~GLMesh() {}

    void clear();
    bool loadObjFile(const std::string& filename, float width = -1);

private:
    std::vector<glm::uvec3> indices;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;

    virtual void drawGL();
    void updateNormals();
    void normalize(float width);  // normalize position and scale
};

class GLLine : public GLBaseShape {
public:
    GLLine() : tail(0.f), head(1.f), width(3.f), head_size(0.f) {}
    ~GLLine() {}

    void setHeadPosition(const glm::vec3& p) { tail = p; }
    void setTailPosition(const glm::vec3& p) { head = p; }
    void setWidth(float w) { width = w; }
    void setHeadSize(float hs) { head_size = hs; }

private:
    glm::vec3 tail, head;
    float width, head_size;

    virtual void drawGL();
};

#endif
