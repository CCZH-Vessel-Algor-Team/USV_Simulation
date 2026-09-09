#include "usv_route_planner/route_planner.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace usv_route_planner
{

namespace
{

struct SearchGrid
{
  int min_row{};
  int min_col{};
  int rows{};
  int cols{};
  double resolution{};
  cv::Mat1f obstacle_distance_m;
  std::vector<uint8_t> common_blocked;
  std::vector<uint8_t> safe_blocked;
  std::vector<int8_t> chart_cost;

  bool contains(const GridIndex & global) const
  {
    return global.row >= min_row && global.col >= min_col &&
           global.row < min_row + rows && global.col < min_col + cols;
  }

  GridIndex local(const GridIndex & global) const
  {
    return {global.row - min_row, global.col - min_col};
  }

  GridIndex global(const GridIndex & local_index) const
  {
    return {local_index.row + min_row, local_index.col + min_col};
  }

  int id(const GridIndex & local_index) const
  {
    return local_index.row * cols + local_index.col;
  }

  bool localContains(const GridIndex & index) const
  {
    return index.row >= 0 && index.col >= 0 && index.row < rows && index.col < cols;
  }

  bool blocked(const GridIndex & index, bool safest) const
  {
    if (!localContains(index)) {
      return true;
    }
    const auto offset = static_cast<std::size_t>(id(index));
    return safest ? safe_blocked[offset] != 0 : common_blocked[offset] != 0;
  }
};

struct SearchResult
{
  bool valid{false};
  std::string message;
  std::vector<GridIndex> path;
  double cost{0.0};
};

double pointDistance(const MapPoint & a, const MapPoint & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

SearchGrid buildSearchGrid(
  const GridMap & map, const std::vector<GridIndex> & relevant_points,
  const PlannerConfig & config, double route_margin_m)
{
  if (relevant_points.empty()) {
    throw std::invalid_argument("Cannot build a search grid without points");
  }

  int min_row = relevant_points.front().row;
  int max_row = min_row;
  int min_col = relevant_points.front().col;
  int max_col = min_col;
  for (const auto & point : relevant_points) {
    if (!map.contains(point)) {
      throw std::out_of_range("A relevant point is outside the map");
    }
    min_row = std::min(min_row, point.row);
    max_row = std::max(max_row, point.row);
    min_col = std::min(min_col, point.col);
    max_col = std::max(max_col, point.col);
  }

  const int route_margin_cells =
    static_cast<int>(std::ceil(route_margin_m / map.resolution()));
  const double total_clearance =
    config.collision_clearance_m + config.safety_hard_extra_clearance_m;
  const int edt_padding_cells =
    static_cast<int>(std::ceil(total_clearance / map.resolution())) + 2;
  const int expansion = route_margin_cells + edt_padding_cells;

  min_row = std::max(0, min_row - expansion);
  max_row = std::min(map.rows() - 1, max_row + expansion);
  min_col = std::max(0, min_col - expansion);
  max_col = std::min(map.cols() - 1, max_col + expansion);

  SearchGrid grid;
  grid.min_row = min_row;
  grid.min_col = min_col;
  grid.rows = max_row - min_row + 1;
  grid.cols = max_col - min_col + 1;
  grid.resolution = map.resolution();

  cv::Mat1b free_image(grid.rows, grid.cols, uint8_t{255});
  grid.chart_cost.resize(static_cast<std::size_t>(grid.rows) * grid.cols);
  for (int row = 0; row < grid.rows; ++row) {
    for (int col = 0; col < grid.cols; ++col) {
      const GridIndex global{row + min_row, col + min_col};
      const int value = map.occupancy(global);
      grid.chart_cost[static_cast<std::size_t>(row) * grid.cols + col] =
        static_cast<int8_t>(value);
      if (map.isRawBlocked(global)) {
        free_image(row, col) = 0;
      }
    }
  }

  // The finite ROI boundary is impassable. EDT padding keeps this artificial
  // boundary away from the start-goal search corridor.
  free_image.row(0).setTo(0);
  free_image.row(grid.rows - 1).setTo(0);
  free_image.col(0).setTo(0);
  free_image.col(grid.cols - 1).setTo(0);

  cv::Mat1f distance_pixels;
  cv::distanceTransform(free_image, distance_pixels, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  grid.obstacle_distance_m = distance_pixels * static_cast<float>(map.resolution());

  const auto cell_count = static_cast<std::size_t>(grid.rows) * grid.cols;
  grid.common_blocked.resize(cell_count);
  grid.safe_blocked.resize(cell_count);
  for (int row = 0; row < grid.rows; ++row) {
    for (int col = 0; col < grid.cols; ++col) {
      const auto offset = static_cast<std::size_t>(row) * grid.cols + col;
      const double distance = grid.obstacle_distance_m(row, col);
      const bool raw_blocked = free_image(row, col) == 0;
      grid.common_blocked[offset] =
        raw_blocked || distance < config.collision_clearance_m;
      grid.safe_blocked[offset] =
        raw_blocked ||
        distance < config.collision_clearance_m + config.safety_hard_extra_clearance_m;
    }
  }
  return grid;
}

bool snapToCommonFree(
  const SearchGrid & grid, const GridIndex & requested, double max_distance_m,
  GridIndex & snapped)
{
  const GridIndex local = grid.local(requested);
  if (!grid.blocked(local, false)) {
    snapped = requested;
    return true;
  }

  const int radius = static_cast<int>(std::ceil(max_distance_m / grid.resolution));
  double best_squared = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int dr = -radius; dr <= radius; ++dr) {
    for (int dc = -radius; dc <= radius; ++dc) {
      const double squared = static_cast<double>(dr * dr + dc * dc);
      if (squared > static_cast<double>(radius * radius) || squared >= best_squared) {
        continue;
      }
      const GridIndex candidate{local.row + dr, local.col + dc};
      if (!grid.blocked(candidate, false)) {
        best_squared = squared;
        snapped = grid.global(candidate);
        found = true;
      }
    }
  }
  return found && std::sqrt(best_squared) * grid.resolution <= max_distance_m;
}

bool lineIsFree(
  const SearchGrid & grid, GridIndex start, const GridIndex & goal,
  bool safest, bool prevent_corner_cutting)
{
  const auto continuousLineIsFree = [&grid, safest](
      double start_row, double start_col, double goal_row, double goal_col)
    {
      const double row_delta = goal_row - start_row;
      const double col_delta = goal_col - start_col;
      const int sample_count = std::max(
        1, static_cast<int>(std::ceil(std::hypot(row_delta, col_delta) * 4.0)));
      for (int i = 0; i <= sample_count; ++i) {
        const double ratio = static_cast<double>(i) / sample_count;
        const GridIndex sample{
          static_cast<int>(std::floor(start_row + ratio * row_delta)),
          static_cast<int>(std::floor(start_col + ratio * col_delta))};
        if (grid.blocked(sample, safest)) {
          return false;
        }
      }
      return true;
    };

  // Sample at sub-cell spacing so every raster cell touched by the geometric
  // segment is checked, including cells a center-based Bresenham line misses.
  if (!continuousLineIsFree(
      start.row + 0.5, start.col + 0.5, goal.row + 0.5, goal.col + 0.5))
  {
    return false;
  }

  int dx = std::abs(goal.col - start.col);
  int sx = start.col < goal.col ? 1 : -1;
  int dy = -std::abs(goal.row - start.row);
  int sy = start.row < goal.row ? 1 : -1;
  int error = dx + dy;

  while (true) {
    if (grid.blocked(start, safest)) {
      return false;
    }
    if (start == goal) {
      return true;
    }
    const GridIndex previous = start;
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      start.col += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      start.row += sy;
    }
    if (prevent_corner_cutting && start.row != previous.row && start.col != previous.col) {
      if (grid.blocked({previous.row, start.col}, safest) ||
        grid.blocked({start.row, previous.col}, safest))
      {
        return false;
      }
    }
  }
}

bool worldLineIsFree(
  const GridMap & map, const SearchGrid & grid, const MapPoint & start,
  const MapPoint & goal, bool safest, double * minimum_obstacle_distance_m = nullptr)
{
  double start_row = 0.0;
  double start_col = 0.0;
  double goal_row = 0.0;
  double goal_col = 0.0;
  if (!map.worldToGridContinuous(start, start_row, start_col) ||
    !map.worldToGridContinuous(goal, goal_row, goal_col))
  {
    return false;
  }
  start_row -= grid.min_row;
  start_col -= grid.min_col;
  goal_row -= grid.min_row;
  goal_col -= grid.min_col;
  const double row_delta = goal_row - start_row;
  const double col_delta = goal_col - start_col;
  const int sample_count = std::max(
    1, static_cast<int>(std::ceil(std::hypot(row_delta, col_delta) * 4.0)));
  for (int i = 0; i <= sample_count; ++i) {
    const double ratio = static_cast<double>(i) / sample_count;
    const GridIndex sample{
      static_cast<int>(std::floor(start_row + ratio * row_delta)),
      static_cast<int>(std::floor(start_col + ratio * col_delta))};
    if (grid.blocked(sample, safest)) {
      return false;
    }
    if (minimum_obstacle_distance_m != nullptr) {
      *minimum_obstacle_distance_m = std::min(
        *minimum_obstacle_distance_m,
        static_cast<double>(grid.obstacle_distance_m(sample.row, sample.col)));
    }
  }
  return true;
}

SearchResult runAStar(
  const SearchGrid & grid, const GridIndex & global_start,
  const GridIndex & global_goal, bool safest, const PlannerConfig & config)
{
  const GridIndex start = grid.local(global_start);
  const GridIndex goal = grid.local(global_goal);
  if (grid.blocked(start, safest) || grid.blocked(goal, safest)) {
    return {false, safest ? "Start or goal violates safe-route clearance" :
           "Start or goal violates collision clearance", {}, 0.0};
  }

  struct QueueEntry
  {
    double f;
    int id;
    bool operator<(const QueueEntry & other) const {return f > other.f;}
  };

  const auto count = static_cast<std::size_t>(grid.rows) * grid.cols;
  std::vector<double> g_score(count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(count, -1);
  std::vector<uint8_t> closed(count, 0);
  std::priority_queue<QueueEntry> open;
  const int start_id = grid.id(start);
  const int goal_id = grid.id(goal);
  g_score[start_id] = 0.0;
  open.push({0.0, start_id});

  const std::array<GridIndex, 8> offsets{{
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}};
  std::size_t expansions = 0;

  while (!open.empty()) {
    const int current_id = open.top().id;
    open.pop();
    if (closed[current_id]) {
      continue;
    }
    closed[current_id] = 1;
    if (++expansions > config.max_expansions) {
      return {false, "A* exceeded max_expansions", {}, 0.0};
    }
    if (current_id == goal_id) {
      break;
    }

    const GridIndex current{current_id / grid.cols, current_id % grid.cols};
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      const bool diagonal = i >= 4;
      if (diagonal && !config.allow_diagonal) {
        continue;
      }
      const GridIndex next{
        current.row + offsets[i].row,
        current.col + offsets[i].col};
      if (grid.blocked(next, safest)) {
        continue;
      }
      if (diagonal && config.prevent_corner_cutting &&
        (grid.blocked({current.row, next.col}, safest) ||
        grid.blocked({next.row, current.col}, safest)))
      {
        continue;
      }

      const int next_id = grid.id(next);
      if (closed[next_id]) {
        continue;
      }
      const double movement = grid.resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      double multiplier = 1.0;
      if (safest) {
        const double distance = grid.obstacle_distance_m(next.row, next.col);
        const double clearance = std::max(0.0, distance - config.collision_clearance_m);
        multiplier += config.safety_weight *
          std::exp(-clearance / config.safety_decay_distance_m);
        const int chart_value = grid.chart_cost[static_cast<std::size_t>(next_id)];
        if (chart_value > 0) {
          multiplier += config.chart_risk_weight *
            std::clamp(static_cast<double>(chart_value) / 100.0, 0.0, 1.0);
        }
      }
      const double tentative = g_score[current_id] + movement * multiplier;
      if (tentative >= g_score[next_id]) {
        continue;
      }
      parent[next_id] = current_id;
      g_score[next_id] = tentative;
      const double heuristic = grid.resolution *
        std::hypot(next.row - goal.row, next.col - goal.col);
      open.push({tentative + heuristic, next_id});
    }
  }

  if (!closed[goal_id]) {
    return {false, safest ? "No safe route in the configured ROI" :
           "No shortest route in the configured ROI", {}, 0.0};
  }

  std::vector<GridIndex> reversed;
  for (int id = goal_id; id >= 0; id = parent[id]) {
    reversed.push_back(grid.global({id / grid.cols, id % grid.cols}));
    if (id == start_id) {
      break;
    }
  }
  std::reverse(reversed.begin(), reversed.end());
  return {true, "OK", std::move(reversed), g_score[goal_id]};
}

std::vector<GridIndex> pruneLineOfSight(
  const SearchGrid & grid, const std::vector<GridIndex> & path,
  bool safest, bool prevent_corner_cutting)
{
  if (path.size() <= 2) {
    return path;
  }
  std::vector<GridIndex> result{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t next = path.size() - 1;
    while (next > anchor + 1 &&
      !lineIsFree(
        grid, grid.local(path[anchor]), grid.local(path[next]), safest,
        prevent_corner_cutting))
    {
      --next;
    }
    result.push_back(path[next]);
    anchor = next;
  }
  return result;
}

std::vector<MapPoint> resamplePath(
  const GridMap & map, const std::vector<GridIndex> & path,
  double requested_spacing, std::size_t max_waypoints)
{
  std::vector<MapPoint> points;
  points.reserve(path.size());
  for (const auto & index : path) {
    points.push_back(map.gridToWorld(index));
  }
  if (points.size() <= 1) {
    return points;
  }

  double total_length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total_length += pointDistance(points[i - 1], points[i]);
  }
  double spacing = requested_spacing;
  const auto outputCount = [&points](double candidate_spacing) {
      std::size_t count = 1;
      for (std::size_t i = 1; i < points.size(); ++i) {
        count += static_cast<std::size_t>(std::ceil(
            pointDistance(points[i - 1], points[i]) / candidate_spacing));
      }
      return count;
    };
  while (outputCount(spacing) > max_waypoints &&
    spacing < total_length && points.size() <= max_waypoints)
  {
    spacing *= 1.25;
  }

  // Every LOS vertex is retained. Dropping a turn and connecting samples on
  // adjacent segments would cut across the obstacle the vertex goes around.
  std::vector<MapPoint> sampled{points.front()};
  for (std::size_t i = 1; i < points.size(); ++i) {
    const MapPoint segment_start = points[i - 1];
    const MapPoint segment_end = points[i];
    const double segment_length = pointDistance(segment_start, segment_end);
    const int subdivisions = std::max(
      1, static_cast<int>(std::ceil(segment_length / spacing)));
    for (int subdivision = 1; subdivision <= subdivisions; ++subdivision) {
      const double ratio = static_cast<double>(subdivision) / subdivisions;
      sampled.push_back({
        segment_start.x + ratio * (segment_end.x - segment_start.x),
        segment_start.y + ratio * (segment_end.y - segment_start.y)});
    }
  }
  return sampled;
}

PlannedRoute finishRoute(
  const GridMap & map, const SearchGrid & grid, const SearchResult & search,
  bool safest, const PlannerConfig & config)
{
  PlannedRoute result;
  result.valid = search.valid;
  result.message = search.message;
  if (!search.valid) {
    return result;
  }

  const auto pruned = pruneLineOfSight(
    grid, search.path, safest, config.prevent_corner_cutting);
  result.map_key_points.reserve(pruned.size());
  for (const auto & index : pruned) {
    result.map_key_points.push_back(map.gridToWorld(index));
  }
  result.map_waypoints = resamplePath(
    map, pruned, config.waypoint_spacing_m, config.max_waypoints);
  double minimum_obstacle_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < result.map_waypoints.size(); ++i) {
    if (!worldLineIsFree(
        map, grid, result.map_waypoints[i - 1], result.map_waypoints[i], safest,
        &minimum_obstacle_distance))
    {
      result.valid = false;
      result.message = "Post-processed path violates the clearance mask";
      result.map_waypoints.clear();
      return result;
    }
  }
  result.weighted_cost = search.cost;
  result.min_clearance_m = std::isfinite(minimum_obstacle_distance) ?
    std::max(0.0, minimum_obstacle_distance - config.collision_clearance_m) : 0.0;
  for (std::size_t i = 1; i < result.map_waypoints.size(); ++i) {
    result.length_m += pointDistance(result.map_waypoints[i - 1], result.map_waypoints[i]);
  }
  return result;
}

}  // namespace

RoutePlanner::RoutePlanner(PlannerConfig config)
: config_(std::move(config))
{
  if (config_.collision_clearance_m < 0.0 ||
    config_.safety_hard_extra_clearance_m < 0.0 ||
    config_.safety_decay_distance_m <= 0.0 ||
    config_.waypoint_spacing_m <= 0.0 || config_.max_waypoints < 2)
  {
    throw std::invalid_argument("Route planner parameters are invalid");
  }
}

PlanPair RoutePlanner::plan(
  const GridMap & map, const MapPoint & start, const MapPoint & goal) const
{
  PlanPair pair;
  GridIndex start_index;
  GridIndex goal_index;
  if (!map.worldToGrid(start, start_index)) {
    std::ostringstream message;
    message << "Start is outside /map: map=(" << start.x << ", " << start.y <<
      "), grid=(row=" << start_index.row << ", col=" << start_index.col <<
      "), map_size=(rows=" << map.rows() << ", cols=" << map.cols() << ")";
    pair.shortest.message = message.str();
    pair.safest.message = pair.shortest.message;
    return pair;
  }
  if (!map.worldToGrid(goal, goal_index)) {
    std::ostringstream message;
    message << "Goal is outside /map: map=(" << goal.x << ", " << goal.y <<
      "), grid=(row=" << goal_index.row << ", col=" << goal_index.col <<
      "), map_size=(rows=" << map.rows() << ", cols=" << map.cols() << ")";
    pair.shortest.message = message.str();
    pair.safest.message = pair.shortest.message;
    return pair;
  }

  const SearchGrid grid = buildSearchGrid(
    map, {start_index, goal_index}, config_, config_.roi_margin_m);
  GridIndex snapped_start;
  GridIndex snapped_goal;
  if (!snapToCommonFree(
      grid, start_index, config_.max_start_goal_snap_distance_m, snapped_start))
  {
    pair.shortest.message = "Start cannot be snapped to collision-free water";
    pair.safest.message = pair.shortest.message;
    return pair;
  }
  if (!snapToCommonFree(
      grid, goal_index, config_.max_start_goal_snap_distance_m, snapped_goal))
  {
    pair.shortest.message = "Goal cannot be snapped to collision-free water";
    pair.safest.message = pair.shortest.message;
    return pair;
  }

  pair.planned_start = map.gridToWorld(snapped_start);
  pair.planned_goal = map.gridToWorld(snapped_goal);
  pair.shortest = finishRoute(
    map, grid, runAStar(grid, snapped_start, snapped_goal, false, config_),
    false, config_);
  pair.safest = finishRoute(
    map, grid, runAStar(grid, snapped_start, snapped_goal, true, config_),
    true, config_);
  return pair;
}

bool RoutePlanner::validatePath(
  const GridMap & map, const std::vector<MapPoint> & path, bool safest,
  std::string & reason) const
{
  if (path.empty()) {
    reason = "Path is empty";
    return false;
  }
  std::vector<GridIndex> indices;
  indices.reserve(path.size());
  for (const auto & point : path) {
    GridIndex index;
    if (!map.worldToGrid(point, index)) {
      reason = "Path leaves /map";
      return false;
    }
    indices.push_back(index);
  }

  const SearchGrid grid = buildSearchGrid(map, indices, config_, 0.0);
  for (std::size_t i = 0; i < indices.size(); ++i) {
    if (grid.blocked(grid.local(indices[i]), safest)) {
      reason = "Path violates the latest static clearance mask";
      return false;
    }
    if (i > 0 && !worldLineIsFree(map, grid, path[i - 1], path[i], safest))
    {
      reason = "A path segment crosses the latest static clearance mask";
      return false;
    }
  }
  reason = "OK";
  return true;
}

}  // namespace usv_route_planner
