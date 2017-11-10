#include "gl_utils.h"

#include <GL/glew.h>
#include <iostream>

// === Error Checker ===
void checkGlError(int idx) {
    GLenum errcode = glGetError();
    if (errcode != GL_NO_ERROR) {
        const GLubyte *errstring = gluErrorString(errcode);
        if (idx >= 0) std::cout << idx;
        std::cout << errstring << std::endl;
    }
}
