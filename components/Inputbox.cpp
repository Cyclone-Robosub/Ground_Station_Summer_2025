#include "imgui.h"
#include <vector>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <climits>
#include <memory>
#include <iostream>
#include "../DashboardGUI.hpp"
// Generic input filter function type
using InputFilter = std::function<bool(char)>;

// Predefined filters
namespace InputFilters
{
    // Float filter: allows digits, decimal point, minus sign, and 'e'/'E' for scientific notation
    static InputFilter Float = [](char c)
    {
        return std::isdigit(c) || c == '.' || c == '-' || c == '+' ||
               c == 'e' || c == 'E' || c == '\b' || c == '\x7f';
    };

    // Integer filter: allows digits and minus sign
    static InputFilter Integer = [](char c)
    {
        return std::isdigit(c) || c == '-' || c == '+' || c == '\b' || c == '\x7f';
    };

    // Alphanumeric filter
    static InputFilter Alphanumeric = [](char c)
    {
        return std::isalnum(c) || c == '\b' || c == '\x7f';
    };

    // No filter (allows everything)
    static InputFilter None = [](char c)
    { return true; };
}

// Input field data structure
struct InputField
{
    std::string descriptor;
    std::string value;
    InputFilter filter;
    size_t maxLength;

    InputField(const std::string &desc, const InputFilter &filt = InputFilters::None, size_t maxLen = 256)
        : descriptor(desc), filter(filt), maxLength(maxLen)
    {
        value.reserve(maxLength);
    }
};

// Custom input text callback for filtering
static int InputTextFilterCallback(ImGuiInputTextCallbackData *data)
{
    InputField *field = static_cast<InputField *>(data->UserData);

    if (data->EventFlag == ImGuiInputTextFlags_CallbackCharFilter)
    {
        // Apply the filter
        if (!field->filter(data->EventChar))
        {
            return 1; // Reject the character
        }
    }

    return 0; // Accept the character
}

// Main widget class
class GenericInputWidget
{
private:
    std::vector<InputField> fields;
    std::string windowTitle;
    bool isOpen;
    ImVec2 windowSize;
    std::shared_ptr<std::array<Axis, 6>> Axes_ptr;

public:
    GenericInputWidget(const std::string &title = "Input Widget", std::shared_ptr<std::array<Axis, 6>> givenAxes_ptr = nullptr)
        : windowTitle(title), isOpen(true), windowSize(800, 500), Axes_ptr(givenAxes_ptr) {}

    // Add a new input field
    void AddField(const std::string &descriptor, const InputFilter &filter = InputFilters::None, size_t maxLength = 256)
    {
    }

    // Remove a field by index
    void RemoveField(size_t index)
    {
    }

    // Clear all fields
    void ClearFields()
    {
    }

    // Render the widget
    bool Render()
    {
        if (!isOpen)
            return false;

        ImGui::SetNextWindowSize(windowSize, ImGuiCond_FirstUseEver);

        if (!ImGui::Begin(windowTitle.c_str(), &isOpen))
        {
            ImGui::End();
            return isOpen;
        }

        // Render input fields
        int b = 0;
        for (Axis axis : (*Axes_ptr))
        {
            if (ImGui::CollapsingHeader(axis.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
            {
                for (size_t i = 0; i < axis.PID_Components.size(); ++i)
                {
                    ImGui::PushID(static_cast<int>(b));

                    // Create a unique label for each field
                    std::string label = axis.PID_Components[i]->name + "##" + std::to_string(i);
                    char givenValue[256];
                    std::unique_lock<std::mutex> UniqueLock(axis.PID_Components[i]->mutex);
                    snprintf(givenValue, sizeof(givenValue), "%.3f", axis.PID_Components[i]->value);

                   UniqueLock.unlock(); 
                    if (ImGui::InputText(axis.PID_Components[i]->name.c_str(), givenValue, sizeof(givenValue), ImGuiInputTextFlags_EnterReturnsTrue))
                    {
                        // Convert string to float on Enter
                        float temp_value = 0.0f;
                        if (sscanf(givenValue, "%f", &temp_value) == 1)
                        {
				std::lock_guard<std::mutex> Lock(axis.PID_Components[i]->mutex);
                            // Only update if the conversion was successful
                            axis.PID_Components[i]->value = temp_value;
                        }
                    }

                    ImGui::PopID();
                    b++;
                }
            }
        }
        if (ImGui::Begin(windowTitle.c_str(), &isOpen))
            ImGui::End();
        return isOpen;
    }

    // Check if window is open
    bool IsOpen() const { return isOpen; }

    // Set window open state
    void SetOpen(bool open) { isOpen = open; }

    // Set window title
    void SetTitle(const std::string &title) { windowTitle = title; }
};

// Example usage function
// void ShowExampleUsage() {
//     static GenericInputWidget widget("My Custom Input Widget");

//     // Initialize with some default fields (only once)
//     static bool initialized = false;
//     if (!initialized) {
//         widget.AddField("Temperature (°C)", InputFilters::Float);
//         widget.AddField("Count", InputFilters::Integer);
//         widget.AddField("Name", InputFilters::Alphanumeric);
//         initialized = true;
//     }

//     // Render the widget
//     widget.Render();

//     // Example of accessing values
//     if (ImGui::Begin("Value Monitor")) {
//         ImGui::Text("Monitoring %zu fields:", widget.GetFieldCount());
//         for (size_t i = 0; i < widget.GetFieldCount(); ++i) {
//             ImGui::Text("Field %zu: '%s'", i, widget.GetFieldValue(i).c_str());
//         }
//     }
//     ImGui::End();
// }

// Custom filter example - only allows hexadecimal characters
// InputFilter CreateHexFilter() {
//     return [](char c) {
//         return std::isdigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F') ||
//                c == '\b' || c == '\x7f';
//     };
// }
