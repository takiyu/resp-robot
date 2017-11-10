#include "gl_shapes.h"

#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader/tiny_obj_loader.h"

#include "render/gl_utils.h"

namespace {
void EnableLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, LIGHT0_POSITION);
}

void DisableLighting() {
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
}
}

// ================================ GLBaseShape ================================
void GLBaseShape::draw() {
    // check the shape is visible
    if (!visibility) return;

    glMatrixMode(GL_MODELVIEW);

    // push
    glPushMatrix();
    glTranslatef(translate[0], translate[1], translate[2]);
    glRotatef(angle[0], 1.f, 0.f, 0.f);  // degree
    glRotatef(angle[1], 0.f, 1.f, 0.f);
    glRotatef(angle[2], 0.f, 0.f, 1.f);
    glScalef(scale, scale, scale);

    // draw
    drawGL();

    // pop
    glPopMatrix();
}

// =================================== GLMesh ==================================
void GLMesh::clear() {
    indices.clear();
    vertices.clear();
    normals.clear();
}

bool GLMesh::loadObjFile(const std::string &filename, float width) {
    clear();
    printf("* Load obj file: %s\n", filename.c_str());
    bool no_normal = false;

    // load
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool ret = tinyobj::LoadObj(shapes, materials, err, filename.c_str());
    if (!ret) {
        printf(" >> Failed to load obj\n");
        return false;
    }
    printf(" >> %lu shapes\n", shapes.size());

    // parse
    for (int shape_idx = 0; shape_idx < shapes.size(); shape_idx++) {
        tinyobj::mesh_t &raw_mesh = shapes[shape_idx].mesh;
        std::vector<unsigned int> &raw_indices = raw_mesh.indices;
        std::vector<float> &raw_vertices = raw_mesh.positions;
        std::vector<float> &raw_normals = raw_mesh.normals;

        if (raw_indices.size() % 3 != 0 ||   // triangle
            raw_vertices.size() % 3 != 0 ||  // xyz
            !(raw_vertices.size() == raw_normals.size() ||
              raw_normals.size() == 0)) {
            printf(" >> Invalid obj format\n");
            return false;
        }

        // parse
        int idx_offset = indices.size();
        int vtx_offset = vertices.size();
        int nml_offset = normals.size();
        indices.resize(idx_offset + raw_indices.size() / 3);
        vertices.resize(vtx_offset + raw_vertices.size() / 3);
        normals.resize(nml_offset + raw_normals.size() / 3);
        // indices
        for (int i = 0; i < raw_indices.size() / 3; i++) {
            indices[idx_offset + i] = glm::make_vec3(&raw_indices[3 * i]);
            indices[idx_offset + i][0] += vtx_offset;
            indices[idx_offset + i][1] += vtx_offset;
            indices[idx_offset + i][2] += vtx_offset;
        }
        // vertices
        for (int i = 0; i < raw_vertices.size() / 3; i++) {
            vertices[vtx_offset + i] = glm::make_vec3(&raw_vertices[3 * i]);
        }
        // normals
        if (raw_normals.size() == raw_vertices.size()) {
            for (int i = 0; i < raw_normals.size() / 3; i++) {
                normals[nml_offset + i] = glm::make_vec3(&raw_normals[3 * i]);
            }
        } else {
            no_normal = true;
            for (int i = 0; i < raw_normals.size() / 3; i++) {
                normals[nml_offset + i] = glm::vec3(0);
            }
        }
    }

    if (no_normal) {
        updateNormals();
    }

    if (width > 0) {
        normalize(width);
    }

    printf(" >> %lu vertices, %lu triangles %s\n", vertices.size(),
           indices.size(), no_normal ? "(normal created)" : "");
    return true;
}

void GLMesh::drawGL() {
    // enable lighting
    EnableLighting();

    // enable material
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glColor3fv(&color[0]);

    // normalize normal
    glEnable(GL_NORMALIZE);

    // draw
    glBegin(GL_TRIANGLES);
    for (int tri = 0; tri < indices.size(); tri++) {
        for (int i = 0; i < 3; i++) {
            unsigned int v_idx = indices[tri][i];
            glNormal3fv(&(normals[v_idx][0]));
            glVertex3fv(&(vertices[v_idx][0]));
        }
    }
    glEnd();

    glDisable(GL_NORMALIZE);

    // disable lighting
    DisableLighting();

    checkGlError();
}

void GLMesh::updateNormals() {
    // initialize normals
    normals.resize(vertices.size());
    for (int i = 0; i < normals.size(); i++) {
        normals[i] = glm::vec3(0.f, 0.f, 0.f);
    }
    // initialize normals weight
    std::vector<int> normals_weight(vertices.size());
    for (int i = 0; i < normals_weight.size(); i++) {
        normals_weight[i] = 0;
    }
    // compute normals based on the faces
    for (int tri_idx = 0; tri_idx < indices.size(); tri_idx++) {
        unsigned int v_idx0 = indices[tri_idx][0];
        unsigned int v_idx1 = indices[tri_idx][1];
        unsigned int v_idx2 = indices[tri_idx][2];
        glm::vec3 v0 = vertices[v_idx0];
        glm::vec3 v1 = vertices[v_idx1];
        glm::vec3 v2 = vertices[v_idx2];
        glm::vec3 e1 = v1 - v0;
        glm::vec3 e2 = v2 - v0;
        glm::vec3 n = glm::normalize(glm::cross(e1, e2));
        // accumulate normals
        normals[v_idx0] += n;
        normals[v_idx1] += n;
        normals[v_idx2] += n;
        // count up
        normals_weight[v_idx0]++;
        normals_weight[v_idx1]++;
        normals_weight[v_idx2]++;
    }
    // average
    for (int v_idx = 0; v_idx < normals.size(); v_idx++) {
        normals[v_idx] /= (float)normals_weight[v_idx];
    }
}

void GLMesh::normalize(float width) {
    // Find current status
    glm::vec3 mean(0.f), max_pt = vertices[0], min_pt = vertices[0];
    for (int i = 0; i < vertices.size(); i++) {
        mean += vertices[i];
        for (int c = 0; c < 3; c++) {
            max_pt[c] = std::max(max_pt[c], vertices[i][c]);
            min_pt[c] = std::min(min_pt[c], vertices[i][c]);
        }
    }
    mean *= 1.f / float(vertices.size());

    // Normalize position
    for (int i = 0; i < vertices.size(); i++) {
        vertices[i] -= mean;
    }
    // Normalize size
    float width_scale = width / (max_pt[0] - min_pt[0]);
    for (int i = 0; i < vertices.size(); i++) {
        vertices[i] *= width_scale;
    }
}

// =================================== GLLine ==================================
void GLLine::drawGL() {
    // color and width
    glColor3fv(&color[0]);
    glLineWidth(width);

    // draw
    glBegin(GL_LINES);
    glVertex3fv(&(tail[0]));
    glVertex3fv(&(head[0]));
    glEnd();

    if (head_size > 0) {
        glPointSize(head_size);
        glBegin(GL_POINTS);
        glVertex3fv(&(head[0]));
        glEnd();
    }

    checkGlError(103);
}
