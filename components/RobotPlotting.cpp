#include <vector>
#include <algorithm> // For std::rotate
#include <math.h>

#include "RobotPlotting.hpp"
#include "implot3d.h"
#include "imgui.h"

void DemoLinePlots() {
    static float xs1[13], ys1[13], zs1[13];
    for (int i = 0; i < 13; i++) {
        xs1[i] = i * 0.001f;
        ys1[i] = 0.5f + 0.5f * cosf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
        zs1[i] = 0.5f + 0.5f * sinf(50 * (xs1[i] + (float)ImGui::GetTime() / 10));
    }
    static double xs2[20], ys2[20], zs2[20];
    for (int i = 0; i < 20; i++) {
        xs2[i] = i * 1 / 19.0f;
        ys2[i] = xs2[i] * xs2[i];
        zs2[i] = xs2[i] * ys2[i];
    }
    if (ImPlot3D::BeginPlot("Line Plots")) {
        ImPlot3D::SetupAxes("x", "y", "z");
        ImPlot3D::PlotLine("f(x)", xs1, ys1, zs1, 13);
        ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Circle);
        ImPlot3D::PlotLine("g(x)", xs2, ys2, zs2, 20, ImPlot3DLineFlags_Segments);
        ImPlot3D::EndPlot();
    }
}

// --- Definition of the constructor ---
// Use the scope resolution operator "::" to specify we're defining a method of LimitedTrajectory
LimitedTrajectory::LimitedTrajectory(size_t max_points) : max_points_(max_points) {
    // Pre-allocate memory to improve performance by avoiding frequent reallocations
    xs_.reserve(max_points_);
    ys_.reserve(max_points_);
    zs_.reserve(max_points_);
}

// --- Definition of the addPoint method ---
void LimitedTrajectory::addPoint(const float* coordinates) {
    if (xs_.size() == max_points_) {
        // If the vector is full, remove the oldest point from the beginning
        xs_.erase(xs_.begin());
        ys_.erase(ys_.begin());
        zs_.erase(zs_.begin());
    }
    xs_.push_back(coordinates[0]);
    ys_.push_back(coordinates[1]);
    zs_.push_back(coordinates[2]);
}

// --- Definition of the plot method ---
void LimitedTrajectory::plot() {
    if (xs_.size() < 2) {
        return; // Nothing to plot
    }
    
    if (ImPlot3D::BeginPlot("Robot Position Trajectory")) {
        ImPlot3D::SetupAxes("x", "y", "z");
        ImPlot3D::PlotLine("Robot Trajectory", xs_.data(), ys_.data(), zs_.data(), xs_.size(), ImPlot3DLineFlags_None);
        ImPlot3D::EndPlot();
    }
}