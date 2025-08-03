#pragma once

#include <string>

//Example - Not for production use
void RenderExampleIndicator(ImGuiIO& io);

// Status code to string conversion
std::string StatusCodeToString(int status_code);

void MasterStatusLight(int& master_status);
void SystemStatusLight(int& system_status);
void RenderStatusIndicators();