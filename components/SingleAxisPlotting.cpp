#include <vector>
#include <algorithm> // For std::rotate
#include <iostream>
#include <math.h>

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

struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};


void DemoLinePlots() {
    static ScrollingBuffer sdata1, sdata2;
    // static RollingBuffer   rdata1, rdata2;
    ImVec2 mouse = ImGui::GetMousePos();
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    sdata1.AddPoint(t, mouse.x * 0.0005f);
    // rdata1.AddPoint(t, mouse.x * 0.0005f);
    sdata2.AddPoint(t, mouse.y * 0.0005f);
    // rdata2.AddPoint(t, mouse.y * 0.0005f);

    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");
    // rdata1.Span = history;
    // rdata2.Span = history;

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
        ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
        ImPlot::PlotShaded("Mouse X", &sdata1.Data[0].x, &sdata1.Data[0].y, sdata1.Data.size(), -INFINITY, 0, sdata1.Offset, 2 * sizeof(float));
        ImPlot::PlotLine("Mouse Y", &sdata2.Data[0].x, &sdata2.Data[0].y, sdata2.Data.size(), 0, sdata2.Offset, 2*sizeof(float));
        ImPlot::EndPlot();
    }
}
