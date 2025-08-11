#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <map>

// Represents a single logged message from a system
struct LoggedMessage {
    std::string text;  // The message content
    std::chrono::system_clock::time_point timestamp; // When the message was logged
};

// Holds the log for a single system (current + prior messages)
struct SystemLog {
    std::string system_name; // Name of the system this log belongs to
    LoggedMessage current_message; // The most recent message
    std::vector<LoggedMessage> prior_messages; // up to 3 previous messages
};

// Utility to add a new message to a system log
void AddSystemMessage(std::map<std::string, SystemLog>& system_logs, const std::string& system_name, const std::string& message_text);

// Render all system logs in a single ImGui window
void RenderMessageLogger(const std::map<std::string, SystemLog>& system_logs);