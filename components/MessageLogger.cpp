#include "MessageLogger.hpp"
#include "imgui.h"
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <ctime>

// Utility: Format timestamp as string
std::string FormatTimestamp(const std::chrono::system_clock::time_point& tp) {
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::tm* tm_ptr = std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", tm_ptr);
    return std::string(buf);
}

// Add a new message to a system log
void AddSystemMessage(std::map<std::string, SystemLog>& system_logs, const std::string& system_name, const std::string& message_text) {
    auto now = std::chrono::system_clock::now();
    LoggedMessage new_msg{message_text, now};

    auto& log = system_logs[system_name];
    log.system_name = system_name;

    // Move current to prior, push new as current
    if (!log.current_message.text.empty()) {
        log.prior_messages.insert(log.prior_messages.begin(), log.current_message);
        if (log.prior_messages.size() > 3)
            log.prior_messages.pop_back();
    }
    log.current_message = new_msg;
}

// Display the most recent message for a system
void DisplayCurrentMessage(const SystemLog& log) {
    ImGui::TextColored(ImVec4(0.8f, 1.0f, 0.8f, 1.0f), "Current: %s", log.current_message.text.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("[%s]", FormatTimestamp(log.current_message.timestamp).c_str());
}

// Display up to three prior messages for a system
void DisplayPriorMessages(const SystemLog& log) {
    for (size_t i = 0; i < log.prior_messages.size(); ++i) {
        ImGui::Text("Previous %zu: %s", i + 1, log.prior_messages[i].text.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("[%s]", FormatTimestamp(log.prior_messages[i].timestamp).c_str());
    }
}

// Render all system logs in a single ImGui window
void RenderMessageLogger(const std::map<std::string, SystemLog>& system_logs) {
    ImGui::Begin("System Message Logger");

    for (const auto& [system_name, log] : system_logs) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "%s", system_name.c_str());
        DisplayCurrentMessage(log);
        DisplayPriorMessages(log);
    }

    ImGui::End();
}