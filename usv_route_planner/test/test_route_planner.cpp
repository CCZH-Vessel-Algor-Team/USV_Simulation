#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "usv_route_planner/grid_map.hpp"
#include "usv_route_planner/route_planner.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid::SharedPtr makeMap(int width, int height)
{
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id = "map";
  map->info.width = width;
  map->info.height = height;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(static_cast<std::size_t>(width * height), 0);
  return map;
}

void setObstacle(nav_msgs::msg::OccupancyGrid & map, int row, int col)
{
  map.data[static_cast<std::size_t>(row) * map.info.width + col] = 100;
}

}  // namespace

TEST(GridMap, AppliesRotatedOrigin)
{
  auto message = makeMap(10, 10);
  message->info.origin.position.x = 10.0;
  message->info.origin.position.y = 20.0;
  const double yaw = M_PI_2;
  message->info.origin.orientation.z = std::sin(yaw / 2.0);
  message->info.origin.orientation.w = std::cos(yaw / 2.0);
  usv_route_planner::GridMap map(message, 80, true);

  const auto world = map.gridToWorld({0, 0});
  EXPECT_NEAR(world.x, 9.5, 1e-9);
  EXPECT_NEAR(world.y, 20.5, 1e-9);
  usv_route_planner::GridIndex index;
  ASSERT_TRUE(map.worldToGrid(world, index));
  EXPECT_EQ(index.row, 0);
  EXPECT_EQ(index.col, 0);
}

TEST(RoutePlanner, InflatesObstacleForBothRoutes)
{
  auto message = makeMap(50, 40);
  setObstacle(*message, 20, 25);
  usv_route_planner::GridMap map(message, 80, true);
  usv_route_planner::PlannerConfig config;
  config.collision_clearance_m = 3.0;
  config.safety_hard_extra_clearance_m = 3.0;
  config.roi_margin_m = 30.0;
  config.waypoint_spacing_m = 1.0;
  config.max_waypoints = 500;
  usv_route_planner::RoutePlanner planner(config);

  const auto result = planner.plan(map, {8.5, 20.5}, {41.5, 20.5});
  ASSERT_TRUE(result.shortest.valid) << result.shortest.message;
  ASSERT_TRUE(result.safest.valid) << result.safest.message;
  ASSERT_GE(result.shortest.map_key_points.size(), 2u);
  EXPECT_LE(result.shortest.map_key_points.size(), result.shortest.map_waypoints.size());
  EXPECT_GE(result.shortest.min_clearance_m, 0.0);
  EXPECT_GE(result.safest.min_clearance_m, 2.9);

  std::string reason;
  EXPECT_TRUE(planner.validatePath(map, result.shortest.map_waypoints, false, reason))
    << reason;
  EXPECT_TRUE(planner.validatePath(map, result.safest.map_waypoints, true, reason))
    << reason;
}

TEST(RoutePlanner, RejectsRawFreePathInsideClearance)
{
  auto message = makeMap(30, 30);
  setObstacle(*message, 15, 15);
  usv_route_planner::GridMap map(message, 80, true);
  usv_route_planner::PlannerConfig config;
  config.collision_clearance_m = 4.0;
  config.safety_hard_extra_clearance_m = 0.0;
  usv_route_planner::RoutePlanner planner(config);

  std::string reason;
  EXPECT_FALSE(planner.validatePath(map, {{10.5, 15.5}, {20.5, 15.5}}, false, reason));
  EXPECT_FALSE(reason.empty());
}

TEST(RoutePlanner, RawObstacleRemainsBlockedWithZeroClearance)
{
  auto message = makeMap(30, 30);
  setObstacle(*message, 15, 15);
  usv_route_planner::GridMap map(message, 80, true);
  usv_route_planner::PlannerConfig config;
  config.collision_clearance_m = 0.0;
  config.safety_hard_extra_clearance_m = 0.0;
  usv_route_planner::RoutePlanner planner(config);

  std::string reason;
  EXPECT_FALSE(planner.validatePath(map, {{10.5, 15.5}, {20.5, 15.5}}, false, reason));
}

TEST(RoutePlanner, ResamplingPreservesCollisionFreeTurns)
{
  auto message = makeMap(60, 50);
  for (int row = 8; row <= 40; ++row) {
    setObstacle(*message, row, 30);
  }
  usv_route_planner::GridMap map(message, 80, true);
  usv_route_planner::PlannerConfig config;
  config.collision_clearance_m = 2.0;
  config.safety_hard_extra_clearance_m = 0.0;
  config.roi_margin_m = 40.0;
  config.waypoint_spacing_m = 20.0;
  config.max_waypoints = 20;
  usv_route_planner::RoutePlanner planner(config);

  const auto result = planner.plan(map, {8.5, 25.5}, {51.5, 25.5});
  ASSERT_TRUE(result.shortest.valid) << result.shortest.message;
  std::string reason;
  EXPECT_TRUE(planner.validatePath(map, result.shortest.map_waypoints, false, reason))
    << reason;
  EXPECT_GT(result.shortest.length_m, 43.0);
}
