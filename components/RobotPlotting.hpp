#pragma once
#include <vector> // For std::vector
#include <cstddef> // For size_t

void DemoLinePlots();

class LimitedTrajectory {
public:
    // --- Declaration of the constructor ---
    LimitedTrajectory(size_t max_points = 2000);

    // --- Declaration of the methods ---
    void addPoint(const float* coordinates);
    void plot();

private:
    size_t max_points_;
    std::vector<float> xs_; // Using a trailing underscore is a common style for private members
    std::vector<float> ys_;
    std::vector<float> zs_;
};