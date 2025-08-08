#include "imgui.h"
#include "BatteryMonitor.hpp"
#include <iostream>

void RenderBatteryMonitor(BatteryStruct &givenBatteryStruct) {
    ImGui::Begin("Battery Monitor");

    // Display the battery voltage
    ImGui::Text("Battery Voltage: %.2f V", givenBatteryStruct.battery_voltage.load());
    //TODO: Add coloring
    ImGui::Text("Battery Status: %s", (givenBatteryStruct.battery_voltage.load() >  givenBatteryStruct.battery_threshold.load()) ? "Good" : "Low");

    // Example voltage history data
    constexpr int history_size = 200;
    static float voltage_history[history_size] = {0};
    static int history_index = 0;

    // Update voltage history with current value
    voltage_history[history_index] = givenBatteryStruct.battery_voltage.load();
    history_index = (history_index + 1) % history_size;

    ImGui::PlotLines("Voltage History", voltage_history, history_size, history_index, nullptr, 0.0f, 16.0f, ImVec2(0, 80));
    // Example of a slider to adjust the battery voltage (for demonstration purposes)
    // ImGui::SliderFloat("Adjust Voltage", &battery_voltage, 0.0f, 12.0f);

    //Battery Current
    static float current_history[history_size] = {0};
    static int current_history_index = 0;
    ImGui::Separator();


    ImGui::Text("Battery Statue of Charge: %.2f %%", givenBatteryStruct.SOC.load(std::memory_order_acquire));
    ImGui::Text("Battery Status: %s", (givenBatteryStruct.SOCint.load(std::memory_order_acquire)) ? "LOW Battery SOC" : "Good");
    current_history[current_history_index] = givenBatteryStruct.SOC.load(std::memory_order_acquire);
    current_history_index = (current_history_index + 1) % history_size;
    ImGui::PlotLines("State of Charge History", current_history, history_size, current_history_index, nullptr, 0.0f, 16.0f, ImVec2(0, 80));
    // std::cout << battery_current << std::endl;

    ImGui::End();
}