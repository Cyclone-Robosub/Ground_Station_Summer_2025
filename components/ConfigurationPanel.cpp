#include "ConfigurationPanel.hpp"
#include "imgui.h"
#include "implot3d.h"

ImVec4 highlightColor;

void RenderConfigurationPanel(ImGuiIO& io, bool& show_demo_window, bool& show_3ddemo_window, bool& dark_mode) {

    if (show_3ddemo_window) {
        ImPlot3D::ShowDemoWindow(&show_3ddemo_window);
    }
  
    if (show_demo_window) {
        ImGui::ShowDemoWindow(&show_demo_window);
    }

    ImGui::Begin("Configuration Panel");
    ImGui::Checkbox("Demo Window", &show_demo_window);     
    ImGui::Checkbox("3D PLotting Demo Window", &show_3ddemo_window);     
    ImGui::Checkbox("Dark Mode", &dark_mode);

    if (dark_mode) {
        ImGui::StyleColorsDark();        
        highlightColor = ImVec4(0.8f, 1.0f, 0.8f, 1.0f); // Light green
    } else {
        ImGui::StyleColorsLight();
        highlightColor = ImVec4(0.0f, 0.4f, 0.0f, 1.0f); // Deep green 
    }

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
}