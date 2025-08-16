#include <math.h>
#include <vector>
#include "SingleAxisPlotting.hpp"
#include "imgui.h"
#include "implot.h"

ScrollingBuffer::ScrollingBuffer(int max_size) {
    MaxSize = max_size;
    Offset = 0;
    Data.reserve(MaxSize);
}

void ScrollingBuffer::AddPoint(float x, float y) {
    if (Data.size() < MaxSize)
        Data.push_back(ImVec2(x, y));
    else {
        Data[Offset] = ImVec2(x, y);
        Offset = (Offset + 1) % MaxSize;
    }
}

void ScrollingBuffer::Erase() {
    if (!Data.empty()) {
        Data.clear();
        Offset = 0;
    }
}

// --- RealTimePlot Method Definitions ---

RealTimePlot::RealTimePlot() {
    // Constructor is empty, but defined here for good practice.
}

void RealTimePlot::AddPosition(float value) {
    DataX.AddPoint(CurrentTime, value);
}

void RealTimePlot::AddWaypoint(float value) {
    DataY.AddPoint(CurrentTime, value);
}

void RealTimePlot::Render(const char* title) {
    // Update the internal time every frame this is drawn.
    CurrentTime += ImGui::GetIO().DeltaTime;

    if (ImPlot::BeginPlot(title, ImVec2(-1, 150))) {
        // Setup axes and limits using the internal CurrentTime.
        ImPlot::SetupAxes(nullptr, nullptr, X_AxisFlags, Y_AxisFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, CurrentTime - History, CurrentTime, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);

        // Ensure there's data to plot to avoid crashes.
        if (!DataX.Data.empty())             {
            // Plot the data from the public buffers.
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            ImPlot::PlotShaded("Position", &DataX.Data[0].x, &DataX.Data[0].y, DataX.Data.size(), -INFINITY, 0, DataX.Offset, sizeof(ImVec2));
        }
        if (!DataY.Data.empty()) {
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            ImPlot::PlotLine("Waypoints", &DataY.Data[0].x, &DataY.Data[0].y, DataY.Data.size(), 0, DataY.Offset, sizeof(ImVec2));
            
        }            

        ImPlot::EndPlot();
    }
}
