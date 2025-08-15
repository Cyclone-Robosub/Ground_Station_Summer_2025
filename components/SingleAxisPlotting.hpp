#pragma once // Prevents the header from being included multiple times

#include "imgui.h"
#include "implot.h"
#include <vector>

// A utility struct for managing scrolling data buffers.
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    std::vector<ImVec2> Data;

    // Constructor declaration
    ScrollingBuffer(int max_size = 2000);
    // Method declarations
    void AddPoint(float x, float y);
    void Erase();
};

// A class to encapsulate the plot's data and rendering logic.
class RealTimePlot {
    
    public:
    // Data buffers for the plot series.
    RealTimePlot(const double axis_min = 0, const double axis_max = 1);

    ScrollingBuffer DataX;
    ScrollingBuffer DataY;
    
    // Public properties to control the plot's appearance.
    float History = 10.0f;
    ImPlotAxisFlags X_AxisFlags = ImPlotAxisFlags_NoTickLabels;
    ImPlotAxisFlags Y_AxisFlags = ImPlotAxisFlags_NoTickLabels;
        
    // Method declarations
    void AddPosition(float value);
    void AddWaypoint(float value);
    void Render(const char* title);

private:
    // This is the internal clock for the plot.
    float CurrentTime_ = 0.0f; 
    double AxisMin_ = 0.0;
    double AxisMax_ = 1.0;

};
