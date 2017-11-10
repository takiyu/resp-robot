#include "gl_imgui.h"

#include <stdio.h>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"

void InitImGui(GLWindow& window) {
    printf("* Initialize ImGui\n");
    ImGui_ImplGlfw_Init(window.getRawRef(), false);
    // Japanese Font
    printf("* Load ImGui font from %s\n", IMGUI_FONT_PATH.c_str());
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontFromFileTTF(IMGUI_FONT_PATH.c_str(), 15.0f, NULL,
                                 io.Fonts->GetGlyphRangesJapanese());
}

void NewImGuiFrame() { ImGui_ImplGlfw_NewFrame(); }

void RenderImGuiFrame() { ImGui::Render(); }

void UpdateImGuiInput(GLWindow& window) {
    ImGuiIO& io = ImGui::GetIO();
    bool in_imgui = io.WantCaptureMouse || io.WantCaptureKeyboard;
    if (in_imgui) {
        window.useInput(false);
        // key
        int key, scancode, action, mods;
        bool key_pushed = window.getKeyStatus(key, scancode, action, mods);
        if (key_pushed) {
            ImGui_ImplGlFw_KeyCallback(window.getRawRef(), key, scancode,
                                       action, mods);
        }
        // char
        unsigned int key_char;
        bool key_char_pushed = window.getCharStatus(key_char);
        if (key_char_pushed) {
            ImGui_ImplGlfw_CharCallback(window.getRawRef(), key_char);
        }
    } else {
        window.useInput(true);
    }
}

void ExitImGui() {
    printf("* Exit ImGui\n");
    ImGui_ImplGlfw_Shutdown();
}
