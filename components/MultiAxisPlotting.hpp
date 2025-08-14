#pragma once
#include <vector> // For std::vector
#include <cstddef> // For size_t
#include <string>

void Demo3DLinePlots();

class LimitedTrajectory {
public:
    LimitedTrajectory(std::string name, size_t max_points = 2000);

    void addPoint(const float* coordinates);
    std::vector <float> getXs() const { return xs_; }
    std::vector <float> getYs() const { return ys_; }
    std::vector <float> getZs() const { return zs_; }

private:
    size_t max_points_;
    std::string name_;
    std::vector<float> xs_;
    std::vector<float> ys_;
    std::vector<float> zs_;
};

void plotLines(const LimitedTrajectory& robot_postition, const LimitedTrajectory& waypoints);