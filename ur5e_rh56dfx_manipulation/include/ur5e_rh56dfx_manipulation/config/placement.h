#pragma once

#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace ur5e_rh56dfx_manipulation
{

// Axis-aligned rectangle on the table, world frame (m)
struct Region
{
  double x_min, x_max, y_min, y_max;

  bool contains(double x, double y) const
  {
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
  }

  // vector of four values [x_min, x_max, y_min, y_max], as stored in the YAML config
  static Region fromVector(const std::vector<double>& v)
  {
    if (v.size() != 4 || v[0] >= v[1] || v[2] >= v[3])
      throw std::invalid_argument("region must be [x_min, x_max, y_min, y_max] with min < max");
    return {v[0], v[1], v[2], v[3]};
  }
};

struct Point2 { double x, y; };

// Centre height of a cube resting on the table (level 0) or on `level` cubes stacked below it
inline double cubeCentreZ(double table_height, double cube_size, int level = 0)
{
  return table_height + cube_size * (level + 0.5);
}

// A point drawn uniformly from `region` that also lies within `max_radius` of the robot base.
// seed 0 draws a different point every call; any other seed always gives the same point.
inline Point2 samplePosition(const Region& region, unsigned seed, double max_radius = 1e9)
{
  std::mt19937 gen(seed ? seed : std::random_device{}());
  std::uniform_real_distribution<> x_dist(region.x_min, region.x_max), y_dist(region.y_min, region.y_max);
  for (int i = 0; i < 1000; ++i) {
    Point2 p{x_dist(gen), y_dist(gen)};
    if (std::hypot(p.x, p.y) <= max_radius) return p;
  }
  throw std::runtime_error("region has no point within max_radius of the base");
}

// Two cubes of edge `cube_size` centred anywhere in a and b can never touch
inline bool regionsSeparated(const Region& a, const Region& b, double cube_size)
{
  return a.x_max + cube_size <= b.x_min || b.x_max + cube_size <= a.x_min ||
         a.y_max + cube_size <= b.y_min || b.y_max + cube_size <= a.y_min;
}

}  // namespace ur5e_rh56dfx_manipulation
