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

// Holds the log for a single system
struct SystemLog {
    std::string system_name;
    LoggedMessage current_message;
    std::vector<LoggedMessage> prior_messages;
};

// Manages all system logs and their rendering
class MessageLogger {
public:
    void AddSystemMessage(const std::string& system_name, const std::string& message_text);
    void Render();

private:
    std::map<std::string, SystemLog> system_logs_;

    // Private helper methods for rendering
    void DisplayCurrentMessage(const SystemLog& log);
    void DisplayPriorMessages(const SystemLog& log);
};