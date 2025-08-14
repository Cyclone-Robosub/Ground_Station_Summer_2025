#include <vector>
#include <algorithm> // For std::rotate
#include <iostream>

#include "SingleAxisPlotting.hpp"
#include "imgui.h"
#include "implot.h"

// #include <memory>

Trajectory::Trajectory(size_t max_points) : max_points_(max_points) {
    // Pre-allocate memory to improve performance by avoiding frequent reallocations
    points_.reserve(max_points_);
}

TrajectoryComparisonPlot::TrajectoryComparisonPlot(std::string name, size_t max_points) : name_(name),  max_points_(max_points) {
    // Pre-allocate memory to improve performance by avoiding frequent reallocations
    waypoints_ = std::make_unique<Trajectory>(max_points_);
    positions_ = std::make_unique<Trajectory>(max_points_);
}

void TrajectoryComparisonPlot::AddWaypoint(const float waypoint) {
    waypoints_->AddPoint(waypoint);
}

void TrajectoryComparisonPlot::AddCurrentPosition(const float position) {
    positions_->AddPoint(position);
}

void Trajectory::AddPoint(const float coordinate) {
    if (points_.size() == max_points_) {
        // If the vector is full, remove the oldest point from the beginning
        points_.erase(points_.begin());
    }
    points_.emplace_back(coordinate, std::chrono::steady_clock::now());
}

std::vector<Point> Trajectory::getPoints() const {
    return points_;
}

Point::Point(float coordinate, std::chrono::steady_clock::time_point timestamp) 
    : coordinate_(coordinate), timestamp_(timestamp) 
{}

float Point::getCoordinate() const {
    // std::cout << "Getting coordinate: " << coordinate_ << std::endl;
    return coordinate_;
}

std::vector<float> Trajectory::getCoordinates() const {
    std::vector<float> coordinates;
    for (const auto& point : points_) {
        coordinates.push_back(point.getCoordinate());
    }
    return coordinates;
}

std::vector<float> Trajectory::getTimestampsSeconds() const {
    std::vector<float> timestamps;
    for (const auto& point : points_) {
        auto duration = point.getTimestamp().time_since_epoch();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
        timestamps.push_back(static_cast<float>(nanoseconds));
    }
    return timestamps;
}


std::chrono::steady_clock::time_point Point::getTimestamp() const {
    return timestamp_;
}

void TrajectoryComparisonPlot::RenderPlot() {
    ImGui::Begin((name_ + " Plot").c_str());
    if (ImPlot::BeginPlot((name_ + " Trajectory").c_str())) {
        // ImPlot::SetupAxes("Time", "Position");
        ImPlot::SetupAxes("Time", "Position", ImPlotAxisFlags_RangeFit, ImPlotAxisFlags_RangeFit);
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Time);
        // ImPlot::SetupAxesLimits(0,100,0,500);
        // ImPlot::SetupAxes("Time", "Position", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        if (!waypoints_->getPoints().empty()) {
            ImPlot::PlotLine("Waypoints", 
            waypoints_->getTimestampsSeconds().data(), 
            waypoints_->getCoordinates().data(), 
            static_cast<int>(waypoints_->getPoints().size()));
          }
          if (!positions_->getPoints().empty()) {
            ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f); // Red line for current position
            ImPlot::PlotLine("Current Position", 
              waypoints_->getTimestampsSeconds().data(), 
              positions_->getCoordinates().data(), 
              static_cast<int>(positions_->getPoints().size()));
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}