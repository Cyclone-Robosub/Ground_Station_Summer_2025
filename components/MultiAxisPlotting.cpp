#include <vector>
#include <algorithm> // For std::rotate
#include <math.h>
#include <iostream>

#include "MultiAxisPlotting.hpp"
#include "implot3d.h"
#include "imgui.h"

void Demo3DLinePlots() {

    // ImPlot3DStyle& style = ImPlot3D::GetStyle();
    // style.LineWeight = 5.0f;
    
    static float xs1[500], ys1[500], zs1[500];
    for (int i = 0; i < 500; i++) {
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
        ImPlot3D::PlotLine("f(x)", xs1, ys1, zs1, 500);
        ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Circle);
        ImPlot3D::PlotLine("g(x)", xs2, ys2, zs2, 20, ImPlot3DLineFlags_Segments);
        ImPlot3D::EndPlot();
    }
}

// --- Definition of the constructor ---
// Use the scope resolution operator "::" to specify we're defining a method of LimitedTrajectory
LimitedTrajectory::LimitedTrajectory(std::string name, size_t max_points) : name_(name),  max_points_(max_points) {
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

Coordinate::Coordinate(float x, float y, float z) : x(x), y(y), z(z) {}

float Coordinate::operator[](int value) const {
    switch(value) {
        case 0:
            return get_x();
        case 1:
            return get_y();
        case 2:
            return get_z();
        default:
            std::cerr << "Index out of bounds! Requested: " << value << "; valid range: [0,5]." << std::endl;
            exit(42);
    }
}

float Coordinate::get_x() const {
    return x;
}

float Coordinate::get_y() const {
    return y;
}

float Coordinate::get_z() const {
    return z;
}

bool my_float_equal(float x, float y) {
    return (x > y) ? (x - y < 0.00001) : (y - x < 0.00001);
}

bool operator==(const Coordinate& lhs, const Coordinate& rhs) {
    return my_float_equal(lhs.x,rhs.x) && my_float_equal(rhs.y,rhs.y) && 
    my_float_equal(lhs.z,rhs.z);
}

bool operator!=(const Coordinate& lhs, const Coordinate& rhs) {
    return !(lhs == rhs);
}


StaticPoint::StaticPoint(std::string name) : name(name), coordinate(Coordinate(0,0,0)), initialized(false) {}

void StaticPoint::addPoint(const Coordinate newCoordinate) {
    if (newCoordinate != this->coordinate) { // only replace point if it's different from the current one
        this->coordinate = newCoordinate;
        initialized = true;
    }
}

void plotLines(const LimitedTrajectory& robot_position, const StaticPoint& waypoint) {

    ImGui::Begin("Robot Trajectory Plots");
    if (ImPlot3D::BeginPlot("Robot Position Trajectory")) {
        ImPlot3D::SetupAxes("x", "y", "z");

        ImPlot3DStyle& style = ImPlot3D::GetStyle();
        style.LineWeight = 2.0f;

        // ImPlot3DNextItemData::LineWeight = 2.0f;
        // &ImPlot3D::style.LineWeight;
        // ImPlot3D::LineWeight(2.0f);
        ImPlot3D::PlotLine("Robot Position", robot_position.getXs().data(), robot_position.getYs().data(), robot_position.getZs().data(),
                           (int)robot_position.getXs().size(), ImPlot3DLineFlags_None);
        ImPlot3D::SetNextMarkerStyle(ImPlot3DMarker_Circle, 4, ImPlot3D::GetColormapColor(1), IMPLOT3D_AUTO, ImPlot3D::GetColormapColor(1));
        ImPlot3D::PushStyleVar(ImPlot3DStyleVar_FillAlpha, 0.60f);
        ImPlot3D::PlotScatter("Waypoint", std::vector<float> {waypoint.getCoordinate()[0]}.data(), std::vector<float> {waypoint.getCoordinate()[1]}.data(), std::vector<float> {waypoint.getCoordinate()[2]}.data(),
                           1, ImPlot3DLineFlags_None);
        ImPlot3D::EndPlot();

    }

    ImGui::End();
}
