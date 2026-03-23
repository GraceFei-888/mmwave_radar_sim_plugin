// mmwave_cloud_colorizer.cpp
//
// ROS 2 node that subscribes to a sensor_msgs/PointCloud2 containing a
// float32 "v" (radial velocity) field and republishes an augmented cloud
// that adds an "rgb" field coloured by velocity:
//
//   v <= v_min  → pure blue  (0, 0, 255)
//   v == 0      → white      (255, 255, 255)
//   v >= v_max  → pure red   (255, 0, 0)
//
// The mapping is piecewise-linear (blue-white-red, "bwr" colormap).
// RViz2 displays the cloud with Color Transformer = RGB8.
//
// Parameters (all declared as ROS 2 node parameters):
//   input_cloud    (string, default "/mmwave/points")
//   output_cloud   (string, default "/mmwave/points_colored")
//   velocity_field (string, default "v")
//   v_min          (double, default -10.0)
//   v_max          (double, default  10.0)
//   colormap       (string, default "bwr")   — only "bwr" is supported
//   clamp          (bool,   default true)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{

// ── Colour mapping ────────────────────────────────────────────────────────────

/// Map a normalised value t ∈ [0, 1] to RGB via blue-white-red.
/// t=0 → blue, t=0.5 → white, t=1 → red.
inline void BwrColor(float t, uint8_t & r, uint8_t & g, uint8_t & b)
{
  t = std::max(0.0f, std::min(1.0f, t));
  if (t <= 0.5f) {
    // blue → white
    float s = t * 2.0f;        // 0..1
    r = static_cast<uint8_t>(255.0f * s);
    g = static_cast<uint8_t>(255.0f * s);
    b = 255;
  } else {
    // white → red
    float s = (t - 0.5f) * 2.0f;  // 0..1
    r = 255;
    g = static_cast<uint8_t>(255.0f * (1.0f - s));
    b = static_cast<uint8_t>(255.0f * (1.0f - s));
  }
}

/// Pack r, g, b into a float32 using PCL/RViz2 convention.
inline float PackRgb(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t rgb = (static_cast<uint32_t>(r) << 16)
    | (static_cast<uint32_t>(g) << 8)
    | static_cast<uint32_t>(b);
  float f;
  std::memcpy(&f, &rgb, sizeof(f));
  return f;
}

// ── Helper: find byte offset of a named field ─────────────────────────────────
int32_t FindFieldOffset(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return static_cast<int32_t>(field.offset);
    }
  }
  return -1;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────

class MmwaveCloudColorizer : public rclcpp::Node
{
public:
  explicit MmwaveCloudColorizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mmwave_cloud_colorizer", options)
  {
    // ── Declare / read parameters ───────────────────────────────────────────
    this->declare_parameter<std::string>("input_cloud",    "/mmwave/points");
    this->declare_parameter<std::string>("output_cloud",   "/mmwave/points_colored");
    this->declare_parameter<std::string>("velocity_field", "v");
    this->declare_parameter<double>("v_min",  -10.0);
    this->declare_parameter<double>("v_max",   10.0);
    this->declare_parameter<std::string>("colormap", "bwr");
    this->declare_parameter<bool>("clamp", true);

    input_topic_    = this->get_parameter("input_cloud").as_string();
    output_topic_   = this->get_parameter("output_cloud").as_string();
    velocity_field_ = this->get_parameter("velocity_field").as_string();
    v_min_          = static_cast<float>(this->get_parameter("v_min").as_double());
    v_max_          = static_cast<float>(this->get_parameter("v_max").as_double());
    clamp_          = this->get_parameter("clamp").as_bool();

    const std::string colormap = this->get_parameter("colormap").as_string();
    if (colormap != "bwr") {
      RCLCPP_WARN(this->get_logger(),
        "Unsupported colormap '%s'. Only 'bwr' (blue-white-red) is supported. "
        "Falling back to 'bwr'.",
        colormap.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
      "mmwave_cloud_colorizer started\n"
      "  input:  %s\n  output: %s\n  field:  %s\n"
      "  v range: [%.2f, %.2f]  clamp: %s",
      input_topic_.c_str(), output_topic_.c_str(), velocity_field_.c_str(),
      v_min_, v_max_, clamp_ ? "true" : "false");

    // ── Publisher / Subscriber ──────────────────────────────────────────────
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->CloudCallback(msg);
      });
  }

private:
  void CloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr in_msg)
  {
    // ── Find the velocity field offset ─────────────────────────────────────
    int32_t v_offset = FindFieldOffset(*in_msg, velocity_field_);
    if (v_offset < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Field '%s' not found in incoming cloud. Skipping.",
        velocity_field_.c_str());
      return;
    }

    // ── Build output cloud = input fields + "rgb" ──────────────────────────
    sensor_msgs::msg::PointCloud2 out_msg;
    out_msg.header     = in_msg->header;
    out_msg.height     = in_msg->height;
    out_msg.width      = in_msg->width;
    out_msg.is_dense   = in_msg->is_dense;
    out_msg.is_bigendian = in_msg->is_bigendian;

    // Copy existing fields
    out_msg.fields = in_msg->fields;

    // Append "rgb" field (float32, packed)
    sensor_msgs::msg::PointField rgb_field;
    rgb_field.name     = "rgb";
    rgb_field.offset   = in_msg->point_step;
    rgb_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    rgb_field.count    = 1;
    out_msg.fields.push_back(rgb_field);

    uint32_t new_point_step = in_msg->point_step + sizeof(float);
    out_msg.point_step = new_point_step;
    out_msg.row_step   = new_point_step * out_msg.width;

    uint32_t num_points = in_msg->width * in_msg->height;
    out_msg.data.resize(static_cast<size_t>(new_point_step) * num_points);

    // ── Copy point data and append rgb ─────────────────────────────────────
    const uint8_t * in_ptr = in_msg->data.data();
    uint8_t * out_ptr = out_msg.data.data();

    const float v_range = v_max_ - v_min_;
    const bool valid_range = (v_range > 1e-6f);

    for (uint32_t i = 0; i < num_points; ++i) {
      // Copy original point bytes
      std::memcpy(out_ptr, in_ptr, in_msg->point_step);

      // Read velocity
      float v_val = 0.0f;
      std::memcpy(&v_val, in_ptr + v_offset, sizeof(float));

      // Map to [0, 1]
      float t = 0.5f;
      if (valid_range) {
        t = (v_val - v_min_) / v_range;
        if (clamp_) {
          t = std::max(0.0f, std::min(1.0f, t));
        }
      }

      // Compute colour
      uint8_t r, g, b;
      BwrColor(t, r, g, b);
      float rgb_packed = PackRgb(r, g, b);

      // Write rgb after original point data
      std::memcpy(out_ptr + in_msg->point_step, &rgb_packed, sizeof(float));

      in_ptr  += in_msg->point_step;
      out_ptr += new_point_step;
    }

    pub_->publish(out_msg);
  }

  // ── Members ────────────────────────────────────────────────────────────────
  std::string input_topic_;
  std::string output_topic_;
  std::string velocity_field_;
  float v_min_{-10.0f};
  float v_max_{10.0f};
  bool  clamp_{true};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MmwaveCloudColorizer>());
  rclcpp::shutdown();
  return 0;
}
