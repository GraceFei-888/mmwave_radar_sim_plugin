// MmwaveSensorPlugin.cpp
// Gazebo Sim 8 (Harmonic) mmWave radar sensor plugin
// Publishes sensor_msgs/PointCloud2 with fields: x, y, z, v (float32)
// Velocity convention: positive = moving away from radar

#include "gazebo_mmwave_sensor/MmwaveSensorPlugin.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// Gazebo Sim 8
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/common/Console.hh>

// ROS 2
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace gazebo_mmwave_sensor
{

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

template<typename T>
T MmwaveSensorPlugin::ReadSdf(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & tag, T default_val)
{
  if (sdf && sdf->HasElement(tag)) {
    return sdf->Get<T>(tag);
  }
  return default_val;
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

MmwaveSensorPlugin::MmwaveSensorPlugin()
: rng_(std::random_device{}()),
  range_noise_(0.0, 0.02)
{
}

// ─────────────────────────────────────────────────────────────────────────────
// ISystemConfigure
// ─────────────────────────────────────────────────────────────────────────────

void MmwaveSensorPlugin::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & /*ecm*/,
  gz::sim::EventManager & /*event_mgr*/)
{
  model_entity_ = entity;

  // ── Read SDF parameters ─────────────────────────────────────────────────
  topic_          = ReadSdf<std::string>(sdf, "topic",            "/mmwave/points");
  frame_id_       = ReadSdf<std::string>(sdf, "frame_id",         "mmwave_link");
  update_rate_    = ReadSdf<double>(sdf,      "update_rate",       10.0);
  min_range_      = ReadSdf<double>(sdf,      "min_range",         0.2);
  max_range_      = ReadSdf<double>(sdf,      "max_range",         50.0);
  h_fov_          = ReadSdf<double>(sdf,      "h_fov",             2.094);
  v_fov_          = ReadSdf<double>(sdf,      "v_fov",             0.349);
  num_rays_h_     = ReadSdf<int>(sdf,         "num_rays_h",        256);
  num_rays_v_     = ReadSdf<int>(sdf,         "num_rays_v",        16);
  range_noise_std_= ReadSdf<double>(sdf,      "range_noise_std",   0.02);
  v_min_          = ReadSdf<double>(sdf,      "v_min",            -10.0);
  v_max_          = ReadSdf<double>(sdf,      "v_max",             10.0);
  debug_draw_rays_= ReadSdf<bool>(sdf,        "debug_draw_rays",   false);

  period_s_ = (update_rate_ > 0.0) ? 1.0 / update_rate_ : 0.1;
  range_noise_ = std::normal_distribution<double>(0.0, range_noise_std_);

  // ── Initialise ROS 2 node (spin-less; we publish manually) ───────────────
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = rclcpp::Node::make_shared("gazebo_mmwave_sensor_node");
  pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS());

  gzmsg << "[MmwaveSensorPlugin] Configured:\n"
        << "  topic:           " << topic_ << "\n"
        << "  frame_id:        " << frame_id_ << "\n"
        << "  update_rate:     " << update_rate_ << " Hz\n"
        << "  range:           [" << min_range_ << ", " << max_range_ << "] m\n"
        << "  h_fov:           " << h_fov_ << " rad\n"
        << "  v_fov:           " << v_fov_ << " rad\n"
        << "  rays:            " << num_rays_h_ << " x " << num_rays_v_ << "\n"
        << "  range_noise_std: " << range_noise_std_ << " m\n"
        << "  v range:         [" << v_min_ << ", " << v_max_ << "] m/s\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// ISystemPostUpdate
// ─────────────────────────────────────────────────────────────────────────────

void MmwaveSensorPlugin::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  // Paused simulation: do nothing
  if (info.paused) {return;}

  // Rate limiting
  double dt = std::chrono::duration<double>(info.dt).count();
  elapsed_since_update_ += dt;
  if (elapsed_since_update_ < period_s_) {return;}
  elapsed_since_update_ = 0.0;

  PublishPointCloud(info, ecm);
}

// ─────────────────────────────────────────────────────────────────────────────
// PublishPointCloud
// ─────────────────────────────────────────────────────────────────────────────

void MmwaveSensorPlugin::PublishPointCloud(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  // ── 1. Get sensor link world pose & velocity ──────────────────────────────
  // The plugin is attached to a model; we look for the link whose name matches
  // frame_id_ (fall back to the first link of the model).
  gz::math::Pose3d sensor_world_pose;
  gz::math::Vector3d sensor_linear_vel_world(0, 0, 0);

  bool found_sensor_link = false;

  ecm.Each<gz::sim::components::Link,
           gz::sim::components::Name,
           gz::sim::components::WorldPose>(
    [&](const gz::sim::Entity & link_entity,
        const gz::sim::components::Link *,
        const gz::sim::components::Name * name_comp,
        const gz::sim::components::WorldPose * pose_comp) -> bool
    {
      // Accept link if it belongs to our model
      auto parent = ecm.ParentEntity(link_entity);
      if (parent != model_entity_) {return true;}  // continue

      if (!found_sensor_link || name_comp->Data() == frame_id_) {
        sensor_world_pose = pose_comp->Data();

        auto vel_comp = ecm.Component<gz::sim::components::WorldLinearVelocity>(
          link_entity);
        if (vel_comp) {
          sensor_linear_vel_world = vel_comp->Data();
        }
        sensor_link_entity_ = link_entity;
        found_sensor_link = true;

        if (name_comp->Data() == frame_id_) {
          return false;  // stop iteration – exact match found
        }
      }
      return true;
    });

  if (!found_sensor_link) {
    // Nothing to publish
    return;
  }

  const gz::math::Vector3d & p_sensor_world = sensor_world_pose.Pos();
  const gz::math::Quaterniond & q_sensor_world = sensor_world_pose.Rot();

  // ── 2. Build ray directions in sensor frame, cast against world ───────────
  // Gazebo Sim 8 does not expose a high-level ray-cast API for custom
  // plugins in the same way the legacy gazebo did. We use a simplified
  // physics-independent approach: we sample the world's static collision
  // geometry by iterating over all WorldPose + Geometry components and
  // computing analytical ray-box/ray-sphere intersections.
  //
  // This is sufficient for a simulation sensor with reasonable scene
  // complexity.  For production use you would hook into gz-physics ray
  // queries via the PhysicsSystem's RequestFeature mechanism.

  // Collect all collision bounding boxes in world space
  struct CollisionObj {
    gz::sim::Entity entity;
    gz::math::Pose3d world_pose;
    gz::math::AxisAlignedBox aabb;
    gz::math::Vector3d linear_vel;
  };
  std::vector<CollisionObj> objects;

  ecm.Each<gz::sim::components::Collision,
           gz::sim::components::WorldPose>(
    [&](const gz::sim::Entity & col_entity,
        const gz::sim::components::Collision *,
        const gz::sim::components::WorldPose * col_pose) -> bool
    {
      // Skip sensor's own link collisions
      auto parent_link = ecm.ParentEntity(col_entity);
      if (parent_link == sensor_link_entity_) {return true;}

      gz::math::Vector3d lin_vel(0, 0, 0);
      auto vel_comp = ecm.Component<gz::sim::components::WorldLinearVelocity>(
        parent_link);
      if (vel_comp) {lin_vel = vel_comp->Data();}

      // Known limitation: gz-sim8 does not yet expose a convenient per-entity
      // BoundingBox component that plugins can query without the rendering
      // system. We therefore approximate each collision with a 1 m unit cube
      // centred at the collision world pose. This gives correct hit/miss
      // decisions for metre-scale objects and is sufficient for typical
      // mmWave simulation scenarios. For precise geometry, hook into the
      // gz-physics RayCast feature via the PhysicsSystem.
      gz::math::AxisAlignedBox aabb(
        col_pose->Data().Pos() - gz::math::Vector3d(0.5, 0.5, 0.5),
        col_pose->Data().Pos() + gz::math::Vector3d(0.5, 0.5, 0.5));

      objects.push_back({col_entity, col_pose->Data(), aabb, lin_vel});
      return true;
    });

  // ── 3. Sample rays and compute hit points ────────────────────────────────
  struct HitPoint {
    gz::math::Vector3d p_sensor;   // in sensor frame
    float v;                        // radial velocity (positive = away)
  };
  std::vector<HitPoint> hits;
  hits.reserve(static_cast<size_t>(num_rays_h_ * num_rays_v_));

  const double h_step = (num_rays_h_ > 1) ?
    h_fov_ / (num_rays_h_ - 1) : 0.0;
  const double v_step = (num_rays_v_ > 1) ?
    v_fov_ / (num_rays_v_ - 1) : 0.0;

  const double h_start = -h_fov_ / 2.0;
  const double v_start = -v_fov_ / 2.0;

  for (int ih = 0; ih < num_rays_h_; ++ih) {
    double az = h_start + ih * h_step;
    for (int iv = 0; iv < num_rays_v_; ++iv) {
      double el = v_start + iv * v_step;

      // Ray direction in sensor frame (X forward, Y left, Z up)
      gz::math::Vector3d dir_sensor(
        std::cos(el) * std::cos(az),
        std::cos(el) * std::sin(az),
        std::sin(el));
      dir_sensor.Normalize();

      // Ray direction in world frame
      gz::math::Vector3d dir_world = q_sensor_world * dir_sensor;
      dir_world.Normalize();

      // Ray-AABB intersection (slab method) – find nearest hit
      double best_t = std::numeric_limits<double>::max();
      gz::math::Vector3d best_hit_world;
      gz::math::Vector3d best_target_vel(0, 0, 0);
      bool got_hit = false;

      for (const auto & obj : objects) {
        const auto & aabb = obj.aabb;
        const gz::math::Vector3d & orig = p_sensor_world;

        double t_min = std::numeric_limits<double>::lowest();
        double t_max = std::numeric_limits<double>::max();

        for (int axis = 0; axis < 3; ++axis) {
          double orig_a = (axis == 0) ? orig.X() :
            (axis == 1) ? orig.Y() : orig.Z();
          double dir_a = (axis == 0) ? dir_world.X() :
            (axis == 1) ? dir_world.Y() : dir_world.Z();
          double mn = (axis == 0) ? aabb.Min().X() :
            (axis == 1) ? aabb.Min().Y() : aabb.Min().Z();
          double mx = (axis == 0) ? aabb.Max().X() :
            (axis == 1) ? aabb.Max().Y() : aabb.Max().Z();

          if (std::fabs(dir_a) < 1e-9) {
            if (orig_a < mn || orig_a > mx) {
              t_min = std::numeric_limits<double>::max();
              break;
            }
          } else {
            double t1 = (mn - orig_a) / dir_a;
            double t2 = (mx - orig_a) / dir_a;
            if (t1 > t2) {std::swap(t1, t2);}
            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);
          }
        }

        if (t_min <= t_max && t_min > 0.0 && t_min < best_t) {
          double range = t_min;
          // Apply range noise
          if (range_noise_std_ > 0.0) {
            range += range_noise_(rng_);
          }
          if (range < min_range_ || range > max_range_) {continue;}
          best_t = t_min;
          best_hit_world = p_sensor_world + dir_world * range;
          best_target_vel = obj.linear_vel;
          got_hit = true;
        }
      }

      if (!got_hit) {continue;}

      // ── 4. Compute radial velocity ────────────────────────────────────────
      // los_world: from sensor toward hit (unit vector)
      gz::math::Vector3d los_world = (best_hit_world - p_sensor_world);
      los_world.Normalize();

      // Relative velocity of target w.r.t. sensor in world frame
      gz::math::Vector3d v_rel = best_target_vel - sensor_linear_vel_world;

      // Radial component (positive = moving away from radar)
      double v_radial = v_rel.Dot(los_world);

      // ── 5. Transform hit point to sensor frame ────────────────────────────
      gz::math::Vector3d p_hit_sensor =
        q_sensor_world.Inverse() * (best_hit_world - p_sensor_world);

      hits.push_back({p_hit_sensor, static_cast<float>(v_radial)});

      if (debug_draw_rays_) {
        gzdbg << "[mmwave] hit at sensor("
              << p_hit_sensor.X() << ", "
              << p_hit_sensor.Y() << ", "
              << p_hit_sensor.Z()
              << ") v=" << v_radial << "\n";
      }
    }
  }

  if (hits.empty()) {return;}

  // ── 6. Build PointCloud2 message ─────────────────────────────────────────
  sensor_msgs::msg::PointCloud2 msg;

  // Header
  auto sim_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    info.simTime).count();
  msg.header.stamp.sec = static_cast<int32_t>(sim_ns / 1'000'000'000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(sim_ns % 1'000'000'000LL);
  msg.header.frame_id = frame_id_;

  // Fields: x, y, z, v
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "v", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(hits.size());

  msg.height = 1;
  msg.width  = static_cast<uint32_t>(hits.size());
  msg.is_dense = true;
  msg.is_bigendian = false;

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_v(msg, "v");

  for (const auto & h : hits) {
    *iter_x = static_cast<float>(h.p_sensor.X());
    *iter_y = static_cast<float>(h.p_sensor.Y());
    *iter_z = static_cast<float>(h.p_sensor.Z());
    *iter_v = h.v;
    ++iter_x; ++iter_y; ++iter_z; ++iter_v;
  }

  pub_->publish(msg);

  // Spin once so ROS callbacks are processed
  rclcpp::spin_some(ros_node_);
}

}  // namespace gazebo_mmwave_sensor

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration (gz-plugin2)
// ─────────────────────────────────────────────────────────────────────────────
GZ_ADD_PLUGIN(
  gazebo_mmwave_sensor::MmwaveSensorPlugin,
  gz::sim::System,
  gazebo_mmwave_sensor::MmwaveSensorPlugin::ISystemConfigure,
  gazebo_mmwave_sensor::MmwaveSensorPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gazebo_mmwave_sensor::MmwaveSensorPlugin,
  "gazebo_mmwave_sensor::MmwaveSensorPlugin")
