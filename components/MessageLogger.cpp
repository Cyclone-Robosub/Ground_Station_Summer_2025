#include "MessageLogger.hpp"
#include "imgui.h"
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <ctime>

// A helper function that formats a timestamp.
// This could be a private method or a free function.
std::string FormatTimestamp(const std::chrono::system_clock::time_point& tp) {
    auto since_epoch = tp.time_since_epoch();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::tm* tm_ptr = std::localtime(&t);

    std::stringstream ss;
    ss << std::put_time(tm_ptr, "%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();

    return ss.str();
}

MessageLogger::MessageLogger(const std::string& system_name)
    : system_name_(system_name) {
    // Initialize the logger with the given system name
    // Initialize(system_name);
}

// Private helper to display the current message
void MessageLogger::DisplayCurrentMessage(const SystemLog& log) {
    auto text_color = ImVec4(0.8f, 1.0f, 0.8f, 1.0f);
    ImGui::TextDisabled("[%s]", FormatTimestamp(log.current_message.timestamp).c_str());
    ImGui::SameLine();
    ImGui::TextColored(text_color, "%s", log.current_message.text.c_str());
}

// Private helper to display prior messages
void MessageLogger::DisplayPriorMessages(const SystemLog& log) {
    for (size_t i = 0; i < log.prior_messages.size(); ++i) {
        ImGui::TextDisabled("[%s]", FormatTimestamp(log.prior_messages[i].timestamp).c_str());
        ImGui::SameLine();
        ImGui::Text("%s", log.prior_messages[i].text.c_str());
    }
}

// Add a new message to a system log
void MessageLogger::AddSystemMessage(const std::string& system_name, const std::string& message_text) {
    auto now = std::chrono::system_clock::now();
    LoggedMessage new_msg{message_text, now};

    auto& log = system_logs_[system_name];
    log.system_name = system_name;

    if (!log.current_message.text.empty()) {
        log.prior_messages.insert(log.prior_messages.begin(), log.current_message);
        if (log.prior_messages.size() > 3) {
            log.prior_messages.pop_back();
        }
    }
    log.current_message = new_msg;
}

// Render all system logs in a single ImGui window
void MessageLogger::Render() {
    ImGui::Begin(system_name_.c_str());

    for (const auto& [system_name, log] : system_logs_) {
        if (ImGui::CollapsingHeader(system_name.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
            DisplayCurrentMessage(log);
            ImGui::Separator();
            DisplayPriorMessages(log);
        }
    }

    ImGui::End();
}