#ifndef USV_ROUTE_PLANNER__GRID_MAP_HPP_
#define USV_ROUTE_PLANNER__GRID_MAP_HPP_

#include <cstdint>
#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace usv_route_planner
{

struct GridIndex
{
  int row{};
  int col{};

  bool operator==(const GridIndex & other) const
  {
    return row == other.row && col == other.col;
  }
};

struct MapPoint
{
  double x{};
  double y{};
};

class GridMap
{
public:
  GridMap(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr message,
    int lethal_threshold, bool unknown_is_obstacle);

  bool worldToGrid(const MapPoint & point, GridIndex & index) const;
  bool worldToGridContinuous(
    const MapPoint & point, double & row, double & col) const;
  MapPoint gridToWorld(const GridIndex & index) const;
  bool contains(const GridIndex & index) const;
  bool isRawBlocked(const GridIndex & index) const;
  int occupancy(const GridIndex & index) const;

  int rows() const;
  int cols() const;
  double resolution() const;
  const nav_msgs::msg::OccupancyGrid & message() const;

private:
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr message_;
  int lethal_threshold_;
  bool unknown_is_obstacle_;
  double origin_yaw_;
};

}  // namespace usv_route_planner

#endif  // USV_ROUTE_PLANNER__GRID_MAP_HPP_
