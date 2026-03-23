#pragma once

#include <memory>
#include <string>
#include <random>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gazebo_mmwave_sensor
{

/// @brief Gazebo Sim 8 System plugin that simulates a mmWave radar sensor.
///
/// The plugin performs 3-D fan-shaped ray sampling, computes radial relative
/// velocity for every hit point, and publishes a sensor_msgs/PointCloud2
/// message with fields x, y, z, v (float32).
///
/// SDF parameters (all optional, defaults shown):
///   <topic>          /mmwave/points
///   <frame_id>       mmwave_link
///   <update_rate>    10          Hz
///   <min_range>      0.2         m
///   <max_range>      50.0        m
///   <h_fov>          2.094       rad (~120 deg)
///   <v_fov>          0.349       rad (~20 deg)
///   <num_rays_h>     256
///   <num_rays_v>     16
///   <range_noise_std> 0.02       m  (0 to disable)
///   <v_min>          -10.0       m/s  (metadata for colorizer)
///   <v_max>           10.0       m/s
///   <debug_draw_rays> false
class MmwaveSensorPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  MmwaveSensorPlugin();
  ~MmwaveSensorPlugin() override = default;

  // ISystemConfigure
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & event_mgr) override;

  // ISystemPostUpdate
  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override;

private:
  // ── Parameters ──────────────────────────────────────────────────────────
  std::string topic_{"mmwave/points"};
  std::string frame_id_{"mmwave_link"};
  double update_rate_{10.0};
  double min_range_{0.2};
  double max_range_{50.0};
  double h_fov_{2.094};
  double v_fov_{0.349};
  int num_rays_h_{256};
  int num_rays_v_{16};
  double range_noise_std_{0.02};
  double v_min_{-10.0};
  double v_max_{10.0};
  bool debug_draw_rays_{false};

  // ── Internal state ───────────────────────────────────────────────────────
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity sensor_link_entity_{gz::sim::kNullEntity};

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  double period_s_{0.1};          // 1 / update_rate
  double elapsed_since_update_{0.0};

  std::mt19937 rng_;
  std::normal_distribution<double> range_noise_;

  // ── Helpers ──────────────────────────────────────────────────────────────
  /// Build and publish the PointCloud2 message.
  void PublishPointCloud(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm);

  /// Read a parameter from SDF with a default fallback.
  template<typename T>
  static T ReadSdf(
    const std::shared_ptr<const sdf::Element> & sdf,
    const std::string & tag, T default_val);
};

}  // namespace gazebo_mmwave_sensor
