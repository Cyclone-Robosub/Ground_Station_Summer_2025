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

bool my_float_equal(float x, float y);

class Coordinate {
    public:
        Coordinate(float x, float y, float z);
        float operator[](int value) const;
        friend bool operator==(const Coordinate& lhs, const Coordinate& rhs);
        friend bool operator!=(const Coordinate& lhs, const Coordinate& rhs);
        float get_x() const;
        float get_y() const;
        float get_z() const;
    private:
        float x, y, z;
};

class StaticPoint {
    public:
        StaticPoint(std::string name);
        void addPoint(const Coordinate coordinate);
        Coordinate getCoordinate() const { return coordinate; }
        bool is_initialized() const { return initialized; }
    private:
        std::string name;
        Coordinate coordinate;
        bool initialized;
};

void plotLines(const LimitedTrajectory& robot_postition, const StaticPoint& waypoint);