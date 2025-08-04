#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <map>

// Represents a single logged message from a system
struct LoggedMessage {
    std::string text;
    std::chrono::system_clock::time_point timestamp;
};

// Holds the log for a single system (current + prior messages)
struct SystemLog {
    std::string system_name;
    LoggedMessage current_message;
    std::vector<LoggedMessage> prior_messages; // up to 3 previous messages
};

// Display the most recent message for a system
void DisplayCurrentMessage(const SystemLog& log);

// Display up to three prior messages for a system
void DisplayPriorMessages(const SystemLog& log);

// Render all system logs in a single ImGui window
void RenderMessageLogger(const std::map<std::string, SystemLog>& system_logs);

// Utility to add a new message to a system log
void AddSystemMessage(std::map<std::string, SystemLog>& system_logs, const std::string& system_name, const std::string& message_text);
