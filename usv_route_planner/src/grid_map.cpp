#include "usv_route_planner/grid_map.hpp"

#include <cmath>
#include <stdexcept>

namespace usv_route_planner
{

namespace
{
double quaternionYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}
}  // namespace

GridMap::GridMap(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr message,
  int lethal_threshold, bool unknown_is_obstacle)
: message_(std::move(message)),
  lethal_threshold_(lethal_threshold),
  unknown_is_obstacle_(unknown_is_obstacle),
  origin_yaw_(0.0)
{
  if (!message_) {
    throw std::invalid_argument("OccupancyGrid message is null");
  }
  const auto expected =
    static_cast<std::size_t>(message_->info.width) * message_->info.height;
  if (message_->info.resolution <= 0.0 || message_->data.size() != expected) {
    throw std::invalid_argument("OccupancyGrid metadata or data size is invalid");
  }
  origin_yaw_ = quaternionYaw(message_->info.origin.orientation);
}

bool GridMap::worldToGrid(const MapPoint & point, GridIndex & index) const
{
  double row = 0.0;
  double col = 0.0;
  const bool inside = worldToGridContinuous(point, row, col);
  index.col = static_cast<int>(std::floor(col));
  index.row = static_cast<int>(std::floor(row));
  return inside;
}

bool GridMap::worldToGridContinuous(
  const MapPoint & point, double & row, double & col) const
{
  const double dx = point.x - message_->info.origin.position.x;
  const double dy = point.y - message_->info.origin.position.y;
  const double c = std::cos(origin_yaw_);
  const double s = std::sin(origin_yaw_);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  col = local_x / resolution();
  row = local_y / resolution();
  return row >= 0.0 && col >= 0.0 && row < rows() && col < cols();
}

MapPoint GridMap::gridToWorld(const GridIndex & index) const
{
  const double local_x = (static_cast<double>(index.col) + 0.5) * resolution();
  const double local_y = (static_cast<double>(index.row) + 0.5) * resolution();
  const double c = std::cos(origin_yaw_);
  const double s = std::sin(origin_yaw_);
  return {
    message_->info.origin.position.x + c * local_x - s * local_y,
    message_->info.origin.position.y + s * local_x + c * local_y};
}

bool GridMap::contains(const GridIndex & index) const
{
  return index.row >= 0 && index.col >= 0 &&
         index.row < rows() && index.col < cols();
}

bool GridMap::isRawBlocked(const GridIndex & index) const
{
  const int value = occupancy(index);
  return value < 0 ? unknown_is_obstacle_ : value >= lethal_threshold_;
}

int GridMap::occupancy(const GridIndex & index) const
{
  if (!contains(index)) {
    return -1;
  }
  const auto offset = static_cast<std::size_t>(index.row) * cols() + index.col;
  return message_->data[offset];
}

int GridMap::rows() const
{
  return static_cast<int>(message_->info.height);
}

int GridMap::cols() const
{
  return static_cast<int>(message_->info.width);
}

double GridMap::resolution() const
{
  return message_->info.resolution;
}

const nav_msgs::msg::OccupancyGrid & GridMap::message() const
{
  return *message_;
}

}  // namespace usv_route_planner
