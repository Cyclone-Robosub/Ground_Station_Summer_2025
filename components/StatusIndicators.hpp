#pragma once

#include <string>

// Example - Not for production use
void RenderExampleIndicator(ImGuiIO& io);

void MasterStatusLight(const std::string& master_status);
void SystemStatusLight(const std::string& system_status);
void RenderStatusIndicators();