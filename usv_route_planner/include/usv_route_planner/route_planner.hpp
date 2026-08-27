#ifndef USV_ROUTE_PLANNER__ROUTE_PLANNER_HPP_
#define USV_ROUTE_PLANNER__ROUTE_PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "usv_route_planner/grid_map.hpp"

namespace usv_route_planner
{

struct PlannerConfig
{
  double collision_clearance_m{20.0};
  double safety_hard_extra_clearance_m{30.0};
  double safety_weight{5.0};
  double safety_decay_distance_m{15.0};
  double chart_risk_weight{1.0};
  double max_start_goal_snap_distance_m{10.0};
  double roi_margin_m{1000.0};
  double waypoint_spacing_m{10.0};
  std::size_t max_waypoints{200};
  std::size_t max_expansions{10000000};
  bool allow_diagonal{true};
  bool prevent_corner_cutting{true};
};

struct PlannedRoute
{
  bool valid{false};
  std::string message;
  std::vector<MapPoint> map_key_points;
  std::vector<MapPoint> map_waypoints;
  double length_m{0.0};
  double weighted_cost{0.0};
  double min_clearance_m{0.0};
};

struct PlanPair
{
  PlannedRoute shortest;
  PlannedRoute safest;
  MapPoint planned_start;
  MapPoint planned_goal;
};

class RoutePlanner
{
public:
  explicit RoutePlanner(PlannerConfig config);

  PlanPair plan(const GridMap & map, const MapPoint & start, const MapPoint & goal) const;
  bool validatePath(
    const GridMap & map, const std::vector<MapPoint> & path, bool safest,
    std::string & reason) const;

private:
  PlannerConfig config_;
};

}  // namespace usv_route_planner

#endif  // USV_ROUTE_PLANNER__ROUTE_PLANNER_HPP_
