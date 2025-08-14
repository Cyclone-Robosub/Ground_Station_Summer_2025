#pragma once
#include "imgui.h"

extern ImVec4 highlightColor;

void RenderConfigurationPanel(ImGuiIO& io, bool& show_demo_window, bool& show_demo_plot, bool& show_3ddemo_window, bool& dark_mode);