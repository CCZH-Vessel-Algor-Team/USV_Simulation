#include <GeographicLib/UTMUPS.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "usv_interfaces/msg/route_candidate.hpp"
#include "usv_interfaces/msg/route_candidates.hpp"
#include "usv_interfaces/msg/route_selection.hpp"
#include "usv_interfaces/msg/waypoint.hpp"
#include "usv_interfaces/msg/waypoint_list.hpp"
#include "usv_route_planner/grid_map.hpp"
#include "usv_route_planner/route_planner.hpp"

namespace usv_route_planner
{

namespace
{
constexpr uint16_t ERROR_OK = 0;
constexpr uint16_t ERROR_MAP_NOT_READY = 1;
constexpr uint16_t ERROR_INVALID_GPS = 2;
constexpr uint16_t ERROR_COORDINATE_TRANSFORM = 3;
constexpr uint16_t ERROR_NO_SHORTEST_ROUTE = 8;

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

bool validGps(double latitude, double longitude)
{
  return std::isfinite(latitude) && std::isfinite(longitude) &&
         latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}
}  // namespace

class RoutePlannerNode : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  RoutePlannerNode()
  : Node("usv_route_planner")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    mission_topic_ =
      declare_parameter<std::string>("mission_topic", "/mission/waypoints");
    candidates_topic_ = declare_parameter<std::string>(
      "candidates_topic", "/route_planner/candidates");
    selection_topic_ = declare_parameter<std::string>(
      "selection_topic", "/route_planner/selection");
    cancel_service_name_ = declare_parameter<std::string>(
      "cancel_service", "/route_planner/cancel");
    map_frame_ = declare_parameter<std::string>("chart_map_frame", "map");
    enforce_map_geometry_ = declare_parameter<bool>("enforce_map_geometry", true);
    expected_map_width_ = declare_parameter<int>("expected_map_width", 12437);
    expected_map_height_ = declare_parameter<int>("expected_map_height", 8689);
    expected_map_resolution_ = declare_parameter<double>(
      "expected_map_resolution", 2.0);
    expected_map_origin_x_ = declare_parameter<double>(
      "expected_map_origin_x", -12437.0);
    expected_map_origin_y_ = declare_parameter<double>(
      "expected_map_origin_y", -8689.0);
    map_geometry_tolerance_ = declare_parameter<double>(
      "map_geometry_tolerance", 1e-3);
    navigate_action_name_ = declare_parameter<std::string>(
      "nav2_action_name", "/navigate_through_poses");
    shortest_path_topic_ = declare_parameter<std::string>(
      "shortest_path_topic", "/route_planner/shortest_path");
    safest_path_topic_ = declare_parameter<std::string>(
      "safest_path_topic", "/route_planner/safest_path");
    selected_path_topic_ = declare_parameter<std::string>(
      "selected_path_topic", "/route_planner/selected_path");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/route_planner/status");

    lethal_threshold_ = declare_parameter<int>("lethal_threshold", 80);
    unknown_is_obstacle_ = declare_parameter<bool>("unknown_is_obstacle", true);
    utm_zone_ = declare_parameter<int>("utm_zone", 50);
    utm_north_ = declare_parameter<bool>("utm_north", true);
    utm_reference_easting_ = declare_parameter<double>(
      "utm_reference_easting", 736235.0);
    utm_reference_northing_ = declare_parameter<double>(
      "utm_reference_northing", 3846381.0);
    default_speed_mps_ = declare_parameter<double>("default_speed_mps", 2.0);
    route_max_age_sec_ = declare_parameter<double>("route_max_age_sec", 60.0);
    nav2_server_timeout_sec_ = declare_parameter<double>(
      "nav2_server_timeout_sec", 2.0);
    execute_with_nav2_ = declare_parameter<bool>("execute_with_nav2", true);
    nav2_include_start_pose_ = declare_parameter<bool>(
      "nav2_include_start_pose", false);
    nav2_behavior_tree_ = declare_parameter<std::string>("nav2_behavior_tree", "");
    nav2_feedback_period_sec_ = declare_parameter<double>(
      "nav2_feedback_period_sec", 1.0);

    PlannerConfig config;
    config.collision_clearance_m = declare_parameter<double>(
      "collision_clearance_m", 20.0);
    config.safety_hard_extra_clearance_m = declare_parameter<double>(
      "safety_hard_extra_clearance_m", 30.0);
    config.safety_weight = declare_parameter<double>("safety_weight", 5.0);
    config.safety_decay_distance_m = declare_parameter<double>(
      "safety_decay_distance_m", 15.0);
    config.chart_risk_weight = declare_parameter<double>("chart_risk_weight", 1.0);
    config.max_start_goal_snap_distance_m = declare_parameter<double>(
      "max_start_goal_snap_distance_m", 10.0);
    config.roi_margin_m = declare_parameter<double>("roi_margin_m", 1000.0);
    config.waypoint_spacing_m = declare_parameter<double>("waypoint_spacing_m", 50.0);
    config.max_waypoints = static_cast<std::size_t>(
      declare_parameter<int>("max_waypoints", 200));
    config.max_expansions = static_cast<std::size_t>(
      declare_parameter<int>("max_expansions", 10000000));
    config.allow_diagonal = declare_parameter<bool>("allow_diagonal", true);
    config.prevent_corner_cutting = declare_parameter<bool>(
      "prevent_corner_cutting", true);
    planner_ = std::make_unique<RoutePlanner>(config);

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&RoutePlannerNode::onMap, this, std::placeholders::_1));
    mission_subscription_ = create_subscription<usv_interfaces::msg::WaypointList>(
      mission_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&RoutePlannerNode::onMission, this, std::placeholders::_1));
    selection_subscription_ = create_subscription<usv_interfaces::msg::RouteSelection>(
      selection_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&RoutePlannerNode::onSelection, this, std::placeholders::_1));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      cancel_service_name_,
      std::bind(
        &RoutePlannerNode::onCancel, this, std::placeholders::_1,
        std::placeholders::_2));

    candidates_publisher_ = create_publisher<usv_interfaces::msg::RouteCandidates>(
      candidates_topic_, rclcpp::QoS(10).reliable());
    shortest_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      shortest_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    safest_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      safest_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    selected_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      selected_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10).reliable());
    nav2_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, navigate_action_name_);

    RCLCPP_INFO(
      get_logger(), "Listening for GPS start/goal on %s and static map on %s",
      mission_topic_.c_str(), map_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Nav2 dispatch: enabled=%s action=%s include_start_pose=%s",
      execute_with_nav2_ ? "true" : "false", navigate_action_name_.c_str(),
      nav2_include_start_pose_ ? "true" : "false");
  }

private:
  struct CachedPlan
  {
    std::string request_id;
    rclcpp::Time created_at;
    usv_interfaces::msg::RouteCandidate shortest;
    usv_interfaces::msg::RouteCandidate safest;
    std::vector<MapPoint> shortest_key_points;
    std::vector<MapPoint> safest_key_points;
  };

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Ignoring /map frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const bool geometry_matches = !enforce_map_geometry_ || (
      (expected_map_width_ <= 0 ||
      message->info.width == static_cast<uint32_t>(expected_map_width_)) &&
      (expected_map_height_ <= 0 ||
      message->info.height == static_cast<uint32_t>(expected_map_height_)) &&
      (expected_map_resolution_ <= 0.0 ||
      std::abs(message->info.resolution - expected_map_resolution_) <=
      map_geometry_tolerance_) &&
      (std::abs(message->info.origin.position.x - expected_map_origin_x_) <=
      map_geometry_tolerance_) &&
      (std::abs(message->info.origin.position.y - expected_map_origin_y_) <=
      map_geometry_tolerance_));
    if (!geometry_matches) {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring incompatible /map: got %ux%u @ %.3f m, origin=(%.3f, %.3f); "
        "expected %dx%d @ %.3f m, origin=(%.3f, %.3f)",
        message->info.width, message->info.height, message->info.resolution,
        message->info.origin.position.x, message->info.origin.position.y,
        expected_map_width_, expected_map_height_, expected_map_resolution_,
        expected_map_origin_x_, expected_map_origin_y_);
      return;
    }
    try {
      (void)GridMap(message, lethal_threshold_, unknown_is_obstacle_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Ignoring invalid /map: %s", error.what());
      return;
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = message;
    RCLCPP_INFO(
      get_logger(), "Cached /map: %ux%u @ %.3f m, origin=(%.3f, %.3f)",
      message->info.width, message->info.height, message->info.resolution,
      message->info.origin.position.x, message->info.origin.position.y);
  }

  MapPoint gpsToMap(double latitude, double longitude) const
  {
    int zone = utm_zone_;
    bool north = utm_north_;
    double easting = 0.0;
    double northing = 0.0;
    GeographicLib::UTMUPS::Forward(
      latitude, longitude, zone, north, easting, northing, utm_zone_);
    if (north != utm_north_) {
      throw std::runtime_error("GPS point is not in the configured UTM hemisphere");
    }
    return {
      easting - utm_reference_easting_,
      northing - utm_reference_northing_};
  }

  usv_interfaces::msg::Waypoint mapToGps(
    const MapPoint & point, double heading) const
  {
    double latitude = 0.0;
    double longitude = 0.0;
    GeographicLib::UTMUPS::Reverse(
      utm_zone_, utm_north_, point.x + utm_reference_easting_,
      point.y + utm_reference_northing_, latitude, longitude);
    usv_interfaces::msg::Waypoint waypoint;
    waypoint.latitude = latitude;
    waypoint.longitude = longitude;
    waypoint.heading_target = heading;
    waypoint.speed_target = static_cast<float>(default_speed_mps_);
    return waypoint;
  }

  nav_msgs::msg::Path makeMapPath(
    const std::vector<MapPoint> & points, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    path.poses.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      double yaw = 0.0;
      if (points.size() > 1) {
        const std::size_t next = i + 1 < points.size() ? i + 1 : i;
        const std::size_t previous = i + 1 < points.size() ? i : i - 1;
        yaw = std::atan2(
          points[next].y - points[previous].y,
          points[next].x - points[previous].x);
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      pose.pose.orientation = quaternionFromYaw(yaw);
      path.poses.push_back(std::move(pose));
    }
    return path;
  }

  usv_interfaces::msg::WaypointList makeGpsPath(
    const std::vector<MapPoint> & points, const rclcpp::Time & stamp) const
  {
    usv_interfaces::msg::WaypointList path;
    path.header.stamp = stamp;
    path.header.frame_id = "wgs84";
    path.waypoints.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      double yaw = 0.0;
      if (points.size() > 1) {
        const std::size_t next = i + 1 < points.size() ? i + 1 : i;
        const std::size_t previous = i + 1 < points.size() ? i : i - 1;
        yaw = std::atan2(
          points[next].y - points[previous].y,
          points[next].x - points[previous].x);
      }
      path.waypoints.push_back(mapToGps(points[i], yaw));
    }
    return path;
  }

  usv_interfaces::msg::RouteCandidate makeCandidate(
    const PlannedRoute & route, uint8_t plan_id, const std::string & request_id,
    const rclcpp::Time & stamp) const
  {
    usv_interfaces::msg::RouteCandidate candidate;
    candidate.header.stamp = stamp;
    candidate.header.frame_id = map_frame_;
    candidate.request_id = request_id;
    candidate.plan_id = plan_id;
    candidate.valid = route.valid;
    candidate.message = route.message;
    candidate.length_m = route.length_m;
    candidate.weighted_cost = route.weighted_cost;
    candidate.min_clearance_m = route.min_clearance_m;
    if (route.valid) {
      candidate.map_path = makeMapPath(route.map_waypoints, stamp);
      candidate.gps_path = makeGpsPath(route.map_key_points, stamp);
    }
    return candidate;
  }

  void publishFailure(uint16_t code, const std::string & message)
  {
    usv_interfaces::msg::RouteCandidates output;
    output.header.stamp = now();
    output.header.frame_id = map_frame_;
    output.error_code = code;
    output.message = message;
    output.shortest.message = message;
    output.safest.message = message;
    candidates_publisher_->publish(output);
    publishStatus(message);
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }

  void onMission(const usv_interfaces::msg::WaypointList::SharedPtr message)
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_message;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_message = latest_map_;
    }
    if (!map_message) {
      publishFailure(ERROR_MAP_NOT_READY, "No valid /map has been received");
      return;
    }
    if (message->waypoints.size() < 2) {
      publishFailure(ERROR_INVALID_GPS, "WaypointList requires at least start and goal");
      return;
    }
    const auto & start_gps = message->waypoints.front();
    const auto & goal_gps = message->waypoints.back();
    if (!validGps(start_gps.latitude, start_gps.longitude) ||
      !validGps(goal_gps.latitude, goal_gps.longitude))
    {
      publishFailure(ERROR_INVALID_GPS, "Start or goal contains invalid WGS84 coordinates");
      return;
    }

    try {
      const MapPoint start = gpsToMap(start_gps.latitude, start_gps.longitude);
      const MapPoint goal = gpsToMap(goal_gps.latitude, goal_gps.longitude);
      const GridMap map(map_message, lethal_threshold_, unknown_is_obstacle_);
      RCLCPP_INFO(
        get_logger(),
        "GPS request converted to map: start=(%.3f, %.3f), goal=(%.3f, %.3f), "
        "map=%ux%u @ %.3f m, origin=(%.3f, %.3f)",
        start.x, start.y, goal.x, goal.y, map_message->info.width,
        map_message->info.height, map_message->info.resolution,
        map_message->info.origin.position.x, map_message->info.origin.position.y);
      const PlanPair result = planner_->plan(map, start, goal);
      const auto stamp = now();
      const std::string request_id = std::to_string(stamp.nanoseconds()) + "-" +
        std::to_string(++plan_sequence_);

      usv_interfaces::msg::RouteCandidates output;
      output.header.stamp = stamp;
      output.header.frame_id = map_frame_;
      output.request_id = request_id;
      output.map_stamp = map_message->header.stamp;
      output.shortest = makeCandidate(
        result.shortest, usv_interfaces::msg::RouteCandidate::PLAN_SHORTEST,
        request_id, stamp);
      output.safest = makeCandidate(
        result.safest, usv_interfaces::msg::RouteCandidate::PLAN_SAFEST,
        request_id, stamp);
      output.error_code = result.shortest.valid ? ERROR_OK : ERROR_NO_SHORTEST_ROUTE;
      output.message = result.shortest.valid ? "OK" : result.shortest.message;
      candidates_publisher_->publish(output);

      if (result.shortest.valid) {
        shortest_path_publisher_->publish(output.shortest.map_path);
      }
      if (result.safest.valid) {
        safest_path_publisher_->publish(output.safest.map_path);
      }

      CachedPlan cache{
        request_id, stamp, output.shortest, output.safest,
        result.shortest.map_key_points, result.safest.map_key_points};
      {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        cached_plan_ = std::make_shared<CachedPlan>(std::move(cache));
      }
      std::ostringstream status;
      status << "request_id=" << request_id << " shortest=" << result.shortest.message <<
        " safest=" << result.safest.message;
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    } catch (const std::exception & error) {
      publishFailure(
        ERROR_COORDINATE_TRANSFORM,
        std::string("Planning failed: ") + error.what());
    }
  }

  void onSelection(const usv_interfaces::msg::RouteSelection::SharedPtr message)
  {
    RCLCPP_INFO(
      get_logger(), "Received route selection: request_id=%s plan_id=%u",
      message->request_id.c_str(), static_cast<unsigned int>(message->plan_id));
    std::shared_ptr<const CachedPlan> plan;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      plan = cached_plan_;
    }
    if (!plan || message->request_id != plan->request_id) {
      publishStatus("Selection rejected: request_id is missing or stale");
      return;
    }
    if ((now() - plan->created_at).seconds() > route_max_age_sec_) {
      publishStatus("Selection rejected: route has expired");
      return;
    }

    const bool safest = message->plan_id ==
      usv_interfaces::msg::RouteSelection::PLAN_SAFEST;
    if (!safest && message->plan_id !=
      usv_interfaces::msg::RouteSelection::PLAN_SHORTEST)
    {
      publishStatus("Selection rejected: plan_id must be 1 or 2");
      return;
    }
    const auto & candidate = safest ? plan->safest : plan->shortest;
    const auto & key_points = safest ?
      plan->safest_key_points : plan->shortest_key_points;
    if (!candidate.valid) {
      publishStatus("Selection rejected: selected candidate is invalid");
      return;
    }

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_message;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_message = latest_map_;
    }
    std::string reason;
    try {
      const GridMap map(map_message, lethal_threshold_, unknown_is_obstacle_);
      if (!planner_->validatePath(map, key_points, safest, reason)) {
        publishStatus("Selection rejected by latest /map: " + reason);
        return;
      }
    } catch (const std::exception & error) {
      publishStatus(std::string("Selection validation failed: ") + error.what());
      return;
    }

    const auto dispatch_stamp = now();
    nav_msgs::msg::Path selected_path = makeMapPath(key_points, dispatch_stamp);
    for (auto & pose : selected_path.poses) {
      if (!std::isfinite(pose.pose.position.x) ||
        !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(pose.pose.orientation.z) ||
        !std::isfinite(pose.pose.orientation.w))
      {
        publishStatus("Selection rejected: selected path contains a non-finite pose");
        return;
      }
    }
    if (selected_path.poses.size() < 2) {
      publishStatus("Selection rejected: selected path requires start and goal");
      return;
    }

    nav_msgs::msg::Path nav2_path = selected_path;
    if (!nav2_include_start_pose_) {
      nav2_path.poses.erase(nav2_path.poses.begin());
    }
    if (nav2_path.poses.empty()) {
      publishStatus("Selection rejected: selected path contains no Nav2 goals");
      return;
    }

    selected_path_publisher_->publish(selected_path);
    std::ostringstream prepared_status;
    prepared_status << "Selected route prepared: request_id=" << message->request_id <<
      " plan_id=" << static_cast<int>(message->plan_id) <<
      " key_points=" << selected_path.poses.size() <<
      " nav2_poses=" << nav2_path.poses.size();
    publishStatus(prepared_status.str());
    RCLCPP_INFO(get_logger(), "%s", prepared_status.str().c_str());

    if (!execute_with_nav2_) {
      publishStatus("Nav2 dispatch disabled by execute_with_nav2=false");
      return;
    }

    if (!nav2_client_->wait_for_action_server(
        std::chrono::duration<double>(nav2_server_timeout_sec_)))
    {
      publishStatus("Selection rejected: NavigateThroughPoses is unavailable");
      RCLCPP_WARN(
        get_logger(), "NavigateThroughPoses action server '%s' is unavailable",
        navigate_action_name_.c_str());
      return;
    }

    NavigateThroughPoses::Goal goal;
    goal.poses = nav2_path.poses;
    goal.behavior_tree = nav2_behavior_tree_;
    auto options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    options.goal_response_callback = [this](const GoalHandle::SharedPtr & handle) {
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          active_goal_ = handle;
        }
        publishStatus(handle ? "Nav2 accepted the selected route" :
          "Nav2 rejected the selected route");
      };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
      {
        const auto feedback_time = now();
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          if (nav2_feedback_period_sec_ > 0.0 &&
            last_feedback_status_time_.nanoseconds() != 0 &&
            (feedback_time - last_feedback_status_time_).seconds() <
            nav2_feedback_period_sec_)
          {
            return;
          }
          last_feedback_status_time_ = feedback_time;
        }
        std::ostringstream status;
        status << "Nav2 executing: distance_remaining=" << feedback->distance_remaining <<
          " poses_remaining=" << feedback->number_of_poses_remaining;
        publishStatus(status.str());
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          active_goal_.reset();
        }
        std::ostringstream status;
        status << "Nav2 finished with result code=" << static_cast<int>(result.code);
        publishStatus(status.str());
      };
    nav2_client_->async_send_goal(goal, options);
    publishStatus("Sending selected route to NavigateThroughPoses");
  }

  void onCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    GoalHandle::SharedPtr goal;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal = active_goal_;
    }
    if (!goal) {
      response->success = false;
      response->message = "No active Nav2 route";
      return;
    }
    nav2_client_->async_cancel_goal(goal);
    response->success = true;
    response->message = "Cancel request sent to Nav2";
    publishStatus(response->message);
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
  }

  std::string map_topic_;
  std::string mission_topic_;
  std::string candidates_topic_;
  std::string selection_topic_;
  std::string cancel_service_name_;
  std::string shortest_path_topic_;
  std::string safest_path_topic_;
  std::string selected_path_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string navigate_action_name_;
  std::string nav2_behavior_tree_;
  bool enforce_map_geometry_;
  int expected_map_width_;
  int expected_map_height_;
  double expected_map_resolution_;
  double expected_map_origin_x_;
  double expected_map_origin_y_;
  double map_geometry_tolerance_;
  int lethal_threshold_;
  bool unknown_is_obstacle_;
  int utm_zone_;
  bool utm_north_;
  double utm_reference_easting_;
  double utm_reference_northing_;
  double default_speed_mps_;
  double route_max_age_sec_;
  double nav2_server_timeout_sec_;
  double nav2_feedback_period_sec_;
  bool execute_with_nav2_;
  bool nav2_include_start_pose_;

  std::unique_ptr<RoutePlanner> planner_;
  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  std::mutex plan_mutex_;
  std::shared_ptr<CachedPlan> cached_plan_;
  std::atomic<uint64_t> plan_sequence_{0};
  std::mutex goal_mutex_;
  GoalHandle::SharedPtr active_goal_;
  std::mutex feedback_mutex_;
  rclcpp::Time last_feedback_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<usv_interfaces::msg::WaypointList>::SharedPtr mission_subscription_;
  rclcpp::Subscription<usv_interfaces::msg::RouteSelection>::SharedPtr
    selection_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<usv_interfaces::msg::RouteCandidates>::SharedPtr
    candidates_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shortest_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safest_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav2_client_;
};

}  // namespace usv_route_planner

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<usv_route_planner::RoutePlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Failed to start route planner: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
