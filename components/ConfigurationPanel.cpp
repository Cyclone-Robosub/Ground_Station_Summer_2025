#include "ConfigurationPanel.hpp"
#include "imgui.h"
#include "implot3d.h"

// The implementation of the GUI function
void RenderConfigurationPanel(ImGuiIO& io, bool& show_demo_window, bool& show_3ddemo_window) {
    // Start the Dear ImGui frame

    // Show the demo window if requested
    if (show_3ddemo_window) {
        ImPlot3D::ShowDemoWindow(&show_3ddemo_window);
    }


    // Show the demo window if requested
  
  if (show_demo_window)
    ImGui::ShowDemoWindow(&show_demo_window);
    
    // static float f = 0.0f;
    // static int counter = 0;

    ImGui::Begin("Configuration Panel");                          // Create a window called "Hello, world!" and append into it.

    // ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
    ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
    ImGui::Checkbox("3D PLotting Demo Window", &show_3ddemo_window);      // Edit bools storing our window open/close state
    // ImGui::Checkbox("Another Window", &show_another_window);

    // ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
    // ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

    // if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        // counter++;
    // ImGui::SameLine();
    // ImGui::Text("counter = %d", counter);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
}