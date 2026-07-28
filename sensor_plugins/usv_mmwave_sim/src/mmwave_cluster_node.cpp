// 订阅仿真 4D 点云，3D DBSCAN 聚类后发布 usv_interfaces/MmwaveTargetArray
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <usv_interfaces/msg/mmwave_target_array.hpp>

namespace
{
struct RadarPoint
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float doppler{0.0f};
  float rcs{0.0f};
};

double dist3d(const RadarPoint &a, const RadarPoint &b)
{
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double dist2d(const RadarPoint &a, const RadarPoint &b)
{
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<int> dbscan(
  const std::vector<RadarPoint> &points,
  double eps,
  int min_points,
  bool use_xy_only)
{
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(n, -1);
  if (n == 0) {
    return labels;
  }

  int cluster_id = 0;
  std::vector<int> neighbors;
  neighbors.reserve(static_cast<size_t>(n));

  auto region_query = [&](int idx) {
    neighbors.clear();
    for (int j = 0; j < n; ++j) {
      const double d = use_xy_only ? dist2d(points[idx], points[j]) : dist3d(points[idx], points[j]);
      if (d <= eps) {
        neighbors.push_back(j);
      }
    }
    return neighbors;
  };

  for (int i = 0; i < n; ++i) {
    if (labels[i] != -1) {
      continue;
    }
    region_query(i);
    if (static_cast<int>(neighbors.size()) < min_points) {
      labels[i] = -1;
      continue;
    }

    labels[i] = cluster_id;
    std::vector<int> seed = neighbors;
    for (size_t s = 0; s < seed.size(); ++s) {
      const int idx = seed[s];
      if (labels[idx] == -1) {
        labels[idx] = cluster_id;
        region_query(idx);
        if (static_cast<int>(neighbors.size()) >= min_points) {
          for (int nb : neighbors) {
            if (std::find(seed.begin(), seed.end(), nb) == seed.end()) {
              seed.push_back(nb);
            }
          }
        }
      } else if (labels[idx] == -2) {
        labels[idx] = cluster_id;
      }
    }
    ++cluster_id;
  }
  return labels;
}

}  // namespace

class MmwaveClusterNode final : public rclcpp::Node
{
public:
  MmwaveClusterNode()
  : Node("mmwave_cluster_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/radar/points_4d");
    output_topic_ = declare_parameter<std::string>("output_topic", "/sim/radar/mmw/objects");
    radar_id_ = declare_parameter<std::string>("radar_id", "front");
    cluster_mode_ = declare_parameter<std::string>("cluster_mode", "xyz");
    cluster_eps_m_ = declare_parameter<double>("cluster_eps_m", 3.0);
    cluster_min_points_ = declare_parameter<int>("cluster_min_points", 3);
    min_rcs_ = declare_parameter<double>("min_rcs", 2.0);
    max_range_m_ = declare_parameter<double>("max_range_m", 300.0);
    enable_doppler_prefilter_ = declare_parameter<bool>("enable_doppler_prefilter", false);
    doppler_prefilter_threshold_ = declare_parameter<double>("doppler_prefilter_threshold", 0.05);
    min_cluster_rcs_mean_ = declare_parameter<double>("min_cluster_rcs_mean", 0.0);
    max_clusters_ = declare_parameter<int>("max_clusters", 0);
    min_size_h_m_ = declare_parameter<double>("min_size_h_m", 0.5);
    output_use_reliable_qos_ = declare_parameter<bool>("output_use_reliable_qos", true);

    use_xy_only_ = (cluster_mode_ == "xy");

    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
    if (output_use_reliable_qos_) {
      pub_qos.reliable();
    } else {
      pub_qos.best_effort();
    }

    pub_ = create_publisher<usv_interfaces::msg::MmwaveTargetArray>(output_topic_, pub_qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MmwaveClusterNode::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "mmwave_cluster: radar=%s in=%s out=%s mode=%s eps=%.2f min_pts=%d min_rcs=%.2f",
      radar_id_.c_str(), input_topic_.c_str(), output_topic_.c_str(), cluster_mode_.c_str(),
      cluster_eps_m_, cluster_min_points_, min_rcs_);
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<RadarPoint> points;
    points.reserve(msg->width * std::max(msg->height, static_cast<uint32_t>(1)));

    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> id(*msg, "doppler_velocity");
      sensor_msgs::PointCloud2ConstIterator<float> ir(*msg, "rcs");

      for (; ix != ix.end(); ++ix, ++iy, ++iz, ++id, ++ir) {
        if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
          continue;
        }
        const double range = std::sqrt(
          static_cast<double>(*ix) * *ix +
          static_cast<double>(*iy) * *iy +
          static_cast<double>(*iz) * *iz);
        if (range < 1e-3 || range > max_range_m_) {
          continue;
        }
        if (static_cast<double>(*ir) < min_rcs_) {
          continue;
        }
        if (enable_doppler_prefilter_ &&
          std::abs(static_cast<double>(*id)) < doppler_prefilter_threshold_)
        {
          continue;
        }
        points.push_back({*ix, *iy, *iz, *id, *ir});
      }
    } catch (const std::runtime_error &e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "PointCloud2 field error: %s", e.what());
      return;
    }

    if (points.empty()) {
      usv_interfaces::msg::MmwaveTargetArray empty;
      empty.header = msg->header;
      pub_->publish(empty);
      return;
    }

    const std::vector<int> labels = dbscan(
      points, cluster_eps_m_, cluster_min_points_, use_xy_only_);

    const int max_label = labels.empty() ?
      -1 :
      *std::max_element(labels.begin(), labels.end());

    struct ClusterStats
    {
      int id{0};
      double cx{0.0};
      double cy{0.0};
      double min_x{std::numeric_limits<double>::max()};
      double max_x{std::numeric_limits<double>::lowest()};
      double min_y{std::numeric_limits<double>::max()};
      double max_y{std::numeric_limits<double>::lowest()};
      double min_z{std::numeric_limits<double>::max()};
      double max_z{std::numeric_limits<double>::lowest()};
      double sum_vx{0.0};
      double sum_vy{0.0};
      double sum_rcs{0.0};
      int count{0};
    };

    std::vector<ClusterStats> clusters(static_cast<size_t>(max_label + 1));
    for (size_t i = 0; i < points.size(); ++i) {
      const int lbl = labels[i];
      if (lbl < 0) {
        continue;
      }
      auto &c = clusters[static_cast<size_t>(lbl)];
      c.id = lbl;
      const double px = points[i].x;
      const double py = points[i].y;
      const double pz = points[i].z;
      const double r = std::max(1e-6, std::sqrt(px * px + py * py + pz * pz));
      const double dop = points[i].doppler;
      c.cx += px;
      c.cy += py;
      c.min_x = std::min(c.min_x, px);
      c.max_x = std::max(c.max_x, px);
      c.min_y = std::min(c.min_y, py);
      c.max_y = std::max(c.max_y, py);
      c.min_z = std::min(c.min_z, pz);
      c.max_z = std::max(c.max_z, pz);
      c.sum_vx += dop * (px / r);
      c.sum_vy += dop * (py / r);
      c.sum_rcs += points[i].rcs;
      ++c.count;
    }

    std::vector<ClusterStats> valid;
    valid.reserve(clusters.size());
    for (auto &c : clusters) {
      if (c.count < cluster_min_points_) {
        continue;
      }
      const double mean_rcs = c.sum_rcs / static_cast<double>(c.count);
      if (min_cluster_rcs_mean_ > 0.0 && mean_rcs < min_cluster_rcs_mean_) {
        continue;
      }
      c.cx /= static_cast<double>(c.count);
      c.cy /= static_cast<double>(c.count);
      valid.push_back(c);
    }

    std::sort(valid.begin(), valid.end(), [](const ClusterStats &a, const ClusterStats &b) {
        const double ra = a.cx * a.cx + a.cy * a.cy;
        const double rb = b.cx * b.cx + b.cy * b.cy;
        return ra < rb;
      });

    if (max_clusters_ > 0 &&
      static_cast<int>(valid.size()) > max_clusters_)
    {
      valid.resize(static_cast<size_t>(max_clusters_));
    }

    usv_interfaces::msg::MmwaveTargetArray out;
    out.header = msg->header;
    out.targets.reserve(valid.size());

    for (const auto &c : valid) {
      usv_interfaces::msg::MmwaveTarget t;
      t.x = c.cx;
      t.y = c.cy;
      t.v_x = c.sum_vx / static_cast<double>(c.count);
      t.v_y = c.sum_vy / static_cast<double>(c.count);
      t.size_w = std::max(0.5, c.max_y - c.min_y);
      t.size_l = std::max(0.5, c.max_x - c.min_x);
      t.size_h = std::max(min_size_h_m_, c.max_z - c.min_z);
      t.snr = c.sum_rcs / static_cast<double>(c.count);
      out.targets.push_back(t);
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string radar_id_;
  std::string cluster_mode_;
  double cluster_eps_m_{3.0};
  int cluster_min_points_{3};
  double min_rcs_{2.0};
  double max_range_m_{300.0};
  bool enable_doppler_prefilter_{false};
  double doppler_prefilter_threshold_{0.05};
  double min_cluster_rcs_mean_{0.0};
  int max_clusters_{0};
  double min_size_h_m_{0.5};
  bool output_use_reliable_qos_{true};
  bool use_xy_only_{false};

  rclcpp::Publisher<usv_interfaces::msg::MmwaveTargetArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MmwaveClusterNode>());
  rclcpp::shutdown();
  return 0;
}
