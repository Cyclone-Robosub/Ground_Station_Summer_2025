#pragma once
#include <vector> // For std::vector
#include <cstddef> // For size_t
#include <string>
#include <memory> // For std::unique_ptr
#include <chrono>

class Point {
public:
    Point(float coordinate, std::chrono::steady_clock::time_point timestamp);
    float getCoordinate() const;
    std::chrono::steady_clock::time_point getTimestamp() const;

private:
    float coordinate_;
    std::chrono::steady_clock::time_point timestamp_;
};

class Trajectory { 
  public:
  Trajectory(size_t max_points);
  void AddPoint(const float coordinate);
  std::vector <Point> getPoints() const;
  std::vector<float> getCoordinates() const;
  std::vector<float> getTimestampsMilliseconds() const;
  
  private:
  size_t max_points_;
  std::vector<Point> points_;
};


class TrajectoryComparisonPlot {
public:
    TrajectoryComparisonPlot(std::string name, size_t max_points = 2000);

    void AddWaypoint(const float waypoint);
    void AddCurrentPosition(const float positions);

    void RenderPlot();
    
private:
  size_t max_points_;
  std::string name_;
  std::unique_ptr<Trajectory> waypoints_;
  std::unique_ptr<Trajectory> positions_;
  
};