#ifndef GL_IMGUI_160707
#define GL_IMGUI_160707

#include <string>
#include "gl_window.h"

const std::string IMGUI_FONT_PATH = "../3rdParty/fonts/Ricty-Regular.ttf";

void InitImGui(GLWindow& window);

void NewImGuiFrame();

void RenderImGuiFrame();

void UpdateImGuiInput(GLWindow& window);

void ExitImGui();

#endif
