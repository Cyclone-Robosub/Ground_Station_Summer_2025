#include "imgui.h"
#include <vector>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>

// Generic input filter function type
using InputFilter = std::function<bool(char)>;

// Predefined filters
namespace InputFilters {
    // Float filter: allows digits, decimal point, minus sign, and 'e'/'E' for scientific notation
    static InputFilter Float = [](char c) {
        return std::isdigit(c) || c == '.' || c == '-' || c == '+' ||
               c == 'e' || c == 'E' || c == '\b' || c == '\x7f';
    };

    // Integer filter: allows digits and minus sign
    static InputFilter Integer = [](char c) {
        return std::isdigit(c) || c == '-' || c == '+' || c == '\b' || c == '\x7f';
    };

    // Alphanumeric filter
    static InputFilter Alphanumeric = [](char c) {
        return std::isalnum(c) || c == '\b' || c == '\x7f';
    };

    // No filter (allows everything)
    static InputFilter None = [](char c) { return true; };
}

// Input field data structure
struct InputField {
    std::string descriptor;
    std::string value;
    InputFilter filter;
    size_t maxLength;

    InputField(const std::string& desc, const InputFilter& filt = InputFilters::None, size_t maxLen = 256)
        : descriptor(desc), filter(filt), maxLength(maxLen) {
        value.reserve(maxLength);
    }
};

// Custom input text callback for filtering
static int InputTextFilterCallback(ImGuiInputTextCallbackData* data) {
    InputField* field = static_cast<InputField*>(data->UserData);

    if (data->EventFlag == ImGuiInputTextFlags_CallbackCharFilter) {
        // Apply the filter
        if (!field->filter(data->EventChar)) {
            return 1; // Reject the character
        }
    }

    return 0; // Accept the character
}

// Main widget class
class GenericInputWidget {
private:
    std::vector<InputField> fields;
    std::string windowTitle;
    bool isOpen;
    ImVec2 windowSize;

public:
    GenericInputWidget(const std::string& title = "Input Widget")
        : windowTitle(title), isOpen(true), windowSize(400, 300) {}

    // Add a new input field
    void AddField(const std::string& descriptor, const InputFilter& filter = InputFilters::None, size_t maxLength = 256) {
        fields.emplace_back(descriptor, filter, maxLength);
    }

    // Remove a field by index
    void RemoveField(size_t index) {
        if (index < fields.size()) {
            fields.erase(fields.begin() + index);
        }
    }

    // Clear all fields
    void ClearFields() {
        fields.clear();
    }

    // Get field value by index
    const std::string& GetFieldValue(size_t index) const {
        static const std::string empty;
        return (index < fields.size()) ? fields[index].value : empty;
    }

    // Set field value by index
    void SetFieldValue(size_t index, const std::string& value) {
        if (index < fields.size()) {
            fields[index].value = value;
        }
    }

    // Get number of fields
    size_t GetFieldCount() const {
        return fields.size();
    }

    // Render the widget
    bool Render() {
        if (!isOpen) return false;

        ImGui::SetNextWindowSize(windowSize, ImGuiCond_FirstUseEver);

        if (!ImGui::Begin(windowTitle.c_str(), &isOpen)) {
            ImGui::End();
            return isOpen;
        }

        // Render input fields
        for (size_t i = 0; i < fields.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));

            // Create a unique label for each field
            std::string label = fields[i].descriptor + "##" + std::to_string(i);

            // Resize the value string buffer if needed
            if (fields[i].value.capacity() < fields[i].maxLength) {
                fields[i].value.reserve(fields[i].maxLength);
            }

            // Render the input field with filtering
            if (ImGui::InputText(label.c_str(),
                               &fields[i].value[0],
                               fields[i].maxLength,
                               ImGuiInputTextFlags_CallbackCharFilter | ImGuiInputTextFlags_CallbackResize,
                               InputTextFilterCallback,
                               &fields[i])) {
                // Handle string resizing for C++ strings
                fields[i].value = std::string(fields[i].value.c_str());
            }

            // Add remove button for each field
            ImGui::SameLine();
            if (ImGui::Button(("Remove##" + std::to_string(i)).c_str())) {
                RemoveField(i);
                ImGui::PopID();
                break; // Exit loop since we modified the vector
            }

            ImGui::PopID();
        }

        // Add separator before control buttons
        if (!fields.empty()) {
            ImGui::Separator();
        }

        // Control buttons
        if (ImGui::Button("Add Float Field")) {
            AddField("Float " + std::to_string(fields.size() + 1), InputFilters::Float);
        }

        ImGui::SameLine();
        if (ImGui::Button("Add Integer Field")) {
            AddField("Integer " + std::to_string(fields.size() + 1), InputFilters::Integer);
        }

        ImGui::SameLine();
        if (ImGui::Button("Add Text Field")) {
            AddField("Text " + std::to_string(fields.size() + 1), InputFilters::Alphanumeric);
        }

        if (ImGui::Button("Clear All")) {
            ClearFields();
        }

        // Display current values (for debugging/demonstration)
        if (!fields.empty()) {
            ImGui::Separator();
            ImGui::Text("Current Values:");
            for (size_t i = 0; i < fields.size(); ++i) {
                ImGui::Text("%s: %s", fields[i].descriptor.c_str(), fields[i].value.c_str());
            }
        }

        ImGui::End();
        return isOpen;
    }

    // Check if window is open
    bool IsOpen() const { return isOpen; }

    // Set window open state
    void SetOpen(bool open) { isOpen = open; }

    // Set window title
    void SetTitle(const std::string& title) { windowTitle = title; }
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