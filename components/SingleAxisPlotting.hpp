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
private:
    // This is the internal clock for the plot.
    float CurrentTime = 0.0f; 

public:
    // Data buffers for the plot series.
    ScrollingBuffer DataX;
    ScrollingBuffer DataY;

    // Public properties to control the plot's appearance.
    float History = 10.0f;
    ImPlotAxisFlags X_AxisFlags = ImPlotAxisFlags_NoTickLabels;
    ImPlotAxisFlags Y_AxisFlags = ImPlotAxisFlags_NoTickLabels;

    // Constructor declaration
    RealTimePlot();

    // Method declarations
    void AddPointX(float value);
    void AddPointY(float value);
    void Render(const char* title);
};
