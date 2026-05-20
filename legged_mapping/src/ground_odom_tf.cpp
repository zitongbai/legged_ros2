#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using RotationTuple = std::array<int, 4>;

constexpr std::array<int, 4> kNextAxis = {1, 2, 0, 1};
constexpr double kEpsilon = 1.0e-9;

const std::unordered_map<std::string, RotationTuple> kAxesToTuple = {
  {"sxyz", {0, 0, 0, 0}},
  {"sxyx", {0, 0, 1, 0}},
  {"sxzy", {0, 1, 0, 0}},
  {"sxzx", {0, 1, 1, 0}},
  {"syzx", {1, 0, 0, 0}},
  {"syzy", {1, 0, 1, 0}},
  {"syxz", {1, 1, 0, 0}},
  {"syxy", {1, 1, 1, 0}},
  {"szxy", {2, 0, 0, 0}},
  {"szxz", {2, 0, 1, 0}},
  {"szyx", {2, 1, 0, 0}},
  {"szyz", {2, 1, 1, 0}},
  {"rzyx", {0, 0, 0, 1}},
  {"rxyx", {0, 0, 1, 1}},
  {"ryzx", {0, 1, 0, 1}},
  {"rxzx", {0, 1, 1, 1}},
  {"rxzy", {1, 0, 0, 1}},
  {"ryzy", {1, 0, 1, 1}},
  {"rzxy", {1, 1, 0, 1}},
  {"ryxy", {1, 1, 1, 1}},
  {"ryxz", {2, 0, 0, 1}},
  {"rzxz", {2, 0, 1, 1}},
  {"rxyz", {2, 1, 0, 1}},
  {"rzyz", {2, 1, 1, 1}},
};

const std::unordered_map<std::string, std::string> kOrderToAxes = {
  {"extrinsic_xyz", "sxyz"},
  {"extrinsic_xyx", "sxyx"},
  {"extrinsic_xzy", "sxzy"},
  {"extrinsic_xzx", "sxzx"},
  {"extrinsic_yzx", "syzx"},
  {"extrinsic_yzy", "syzy"},
  {"extrinsic_yxz", "syxz"},
  {"extrinsic_yxy", "syxy"},
  {"extrinsic_zxy", "szxy"},
  {"extrinsic_zxz", "szxz"},
  {"extrinsic_zyx", "szyx"},
  {"extrinsic_zyz", "szyz"},
  {"intrinsic_zyx", "rzyx"},
  {"intrinsic_xyx", "rxyx"},
  {"intrinsic_yzx", "ryzx"},
  {"intrinsic_xzx", "rxzx"},
  {"intrinsic_xzy", "rxzy"},
  {"intrinsic_yzy", "ryzy"},
  {"intrinsic_zxy", "rzxy"},
  {"intrinsic_yxy", "ryxy"},
  {"intrinsic_yxz", "ryxz"},
  {"intrinsic_zxz", "rzxz"},
  {"intrinsic_xyz", "rxyz"},
  {"intrinsic_zyz", "rzyz"},
};

struct TransformConfig
{
  std::string parent_frame;
  std::string child_frame;
  std::vector<double> translation_xyz;
  std::vector<double> rotation_angles;
};

struct RotationConfig
{
  std::string rotation_order;
  std::string angle_unit;
};

struct GroundOdomConfig
{
  std::string parent_frame;
  std::string child_frame;
  std::string base_frame;
  std::vector<std::string> foot_frames;
  double foot_radius;
  double tf_timeout_sec;
};

void validate_vector(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(parameter_name + " must contain exactly 3 values.");
  }

  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(parameter_name + " must contain only finite values.");
    }
  }
}

std::vector<double> convert_angles_to_rad(
  const std::vector<double> & angles,
  const std::string & angle_unit)
{
  if (angle_unit == "rad") {
    return angles;
  }
  if (angle_unit != "deg") {
    throw std::invalid_argument(
            "Unsupported angle_unit '" + angle_unit + "'. Use 'rad' or 'deg'.");
  }

  constexpr double kDegToRad = M_PI / 180.0;
  std::vector<double> angles_rad;
  angles_rad.reserve(angles.size());
  for (const double angle : angles) {
    angles_rad.push_back(angle * kDegToRad);
  }
  return angles_rad;
}

RotationTuple get_rotation_tuple(const std::string & rotation_order)
{
  const auto order_it = kOrderToAxes.find(rotation_order);
  if (order_it == kOrderToAxes.end()) {
    throw std::invalid_argument("Unsupported rotation_order '" + rotation_order + "'.");
  }

  const auto axes_it = kAxesToTuple.find(order_it->second);
  if (axes_it == kAxesToTuple.end()) {
    throw std::invalid_argument(
            "Internal error: missing axes mapping for rotation_order '" +
            rotation_order + "'.");
  }

  return axes_it->second;
}

tf2::Matrix3x3 euler_to_matrix(
  const std::vector<double> & angles_rad,
  const std::string & rotation_order)
{
  const auto rotation_tuple = get_rotation_tuple(rotation_order);
  int i = rotation_tuple[0];
  int parity = rotation_tuple[1];
  int repetition = rotation_tuple[2];
  int frame = rotation_tuple[3];
  int j = kNextAxis[static_cast<std::size_t>(i + parity)];
  int k = kNextAxis[static_cast<std::size_t>(i - parity + 1)];

  double ai = angles_rad[0];
  double aj = angles_rad[1];
  double ak = angles_rad[2];

  if (frame != 0) {
    std::swap(ai, ak);
  }
  if (parity != 0) {
    ai = -ai;
    aj = -aj;
    ak = -ak;
  }

  const double si = std::sin(ai);
  const double sj = std::sin(aj);
  const double sk = std::sin(ak);
  const double ci = std::cos(ai);
  const double cj = std::cos(aj);
  const double ck = std::cos(ak);
  const double cc = ci * ck;
  const double cs = ci * sk;
  const double sc = si * ck;
  const double ss = si * sk;

  std::array<std::array<double, 3>, 3> matrix = {{
    {{1.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}},
  }};

  if (repetition != 0) {
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = cj;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = sj * si;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] = sj * ci;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = sj * sk;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] = -cj * ss + cc;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)] = -cj * cs - sc;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = -sj * ck;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)] = cj * sc + cs;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(k)] = cj * cc - ss;
  } else {
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = cj * ck;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = sj * sc - cs;
    matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] = sj * cc + ss;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] = cj * sk;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] = sj * ss + cc;
    matrix[static_cast<std::size_t>(j)][static_cast<std::size_t>(k)] = sj * cs - sc;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] = -sj;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)] = cj * si;
    matrix[static_cast<std::size_t>(k)][static_cast<std::size_t>(k)] = cj * ci;
  }

  return tf2::Matrix3x3(
    matrix[0][0], matrix[0][1], matrix[0][2],
    matrix[1][0], matrix[1][1], matrix[1][2],
    matrix[2][0], matrix[2][1], matrix[2][2]);
}

RotationConfig load_rotation_config(rclcpp::Node & node)
{
  RotationConfig config;
  config.rotation_order = node.declare_parameter<std::string>("rotation_order", "");
  config.angle_unit = node.declare_parameter<std::string>("angle_unit", "");

  if (config.rotation_order.empty()) {
    throw std::invalid_argument("rotation_order must not be empty.");
  }
  if (config.angle_unit.empty()) {
    throw std::invalid_argument("angle_unit must not be empty.");
  }
  if (config.angle_unit != "rad" && config.angle_unit != "deg") {
    throw std::invalid_argument(
            "Unsupported angle_unit '" + config.angle_unit + "'. Use 'rad' or 'deg'.");
  }

  static_cast<void>(get_rotation_tuple(config.rotation_order));
  return config;
}

TransformConfig load_transform_config(
  rclcpp::Node & node,
  const std::string & namespace_prefix)
{
  const auto parent_param = namespace_prefix + ".parent_frame";
  const auto child_param = namespace_prefix + ".child_frame";
  const auto translation_param = namespace_prefix + ".translation_xyz";
  const auto rotation_param = namespace_prefix + ".rotation_angles";

  TransformConfig config;
  config.parent_frame = node.declare_parameter<std::string>(parent_param, "");
  config.child_frame = node.declare_parameter<std::string>(child_param, "");
  config.translation_xyz =
    node.declare_parameter<std::vector<double>>(translation_param, {0.0, 0.0, 0.0});
  config.rotation_angles =
    node.declare_parameter<std::vector<double>>(rotation_param, {0.0, 0.0, 0.0});

  if (config.parent_frame.empty()) {
    throw std::invalid_argument(parent_param + " must not be empty.");
  }
  if (config.child_frame.empty()) {
    throw std::invalid_argument(child_param + " must not be empty.");
  }
  if (config.parent_frame == config.child_frame) {
    throw std::invalid_argument(
            namespace_prefix + " parent and child frames must be different.");
  }

  validate_vector(config.translation_xyz, translation_param);
  validate_vector(config.rotation_angles, rotation_param);
  return config;
}

GroundOdomConfig load_ground_odom_config(rclcpp::Node & node)
{
  GroundOdomConfig config;
  config.parent_frame =
    node.declare_parameter<std::string>("ground_odom.parent_frame", "");
  config.child_frame =
    node.declare_parameter<std::string>("ground_odom.child_frame", "");
  config.base_frame =
    node.declare_parameter<std::string>("ground_odom.base_frame", "");
  config.foot_frames =
    node.declare_parameter<std::vector<std::string>>(
    "ground_odom.foot_frames", std::vector<std::string>{});
  config.foot_radius =
    node.declare_parameter<double>("ground_odom.foot_radius", 0.0);
  config.tf_timeout_sec =
    node.declare_parameter<double>("ground_odom.tf_timeout_sec", 10.0);

  if (config.parent_frame.empty()) {
    throw std::invalid_argument("ground_odom.parent_frame must not be empty.");
  }
  if (config.child_frame.empty()) {
    throw std::invalid_argument("ground_odom.child_frame must not be empty.");
  }
  if (config.base_frame.empty()) {
    throw std::invalid_argument("ground_odom.base_frame must not be empty.");
  }
  if (config.parent_frame == config.child_frame) {
    throw std::invalid_argument("ground_odom parent and child frames must be different.");
  }
  if (config.foot_frames.size() < 3) {
    throw std::invalid_argument("ground_odom.foot_frames must contain at least 3 frames.");
  }
  for (const auto & foot_frame : config.foot_frames) {
    if (foot_frame.empty()) {
      throw std::invalid_argument("ground_odom.foot_frames must not contain empty names.");
    }
  }
  if (!std::isfinite(config.foot_radius) || config.foot_radius < 0.0) {
    throw std::invalid_argument("ground_odom.foot_radius must be a finite non-negative value.");
  }
  if (!std::isfinite(config.tf_timeout_sec) || config.tf_timeout_sec <= 0.0) {
    throw std::invalid_argument("ground_odom.tf_timeout_sec must be a finite positive value.");
  }

  return config;
}

geometry_msgs::msg::TransformStamped to_transform_stamped(
  const rclcpp::Time & stamp,
  const TransformConfig & config,
  const RotationConfig & rotation_config)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = config.parent_frame;
  transform.child_frame_id = config.child_frame;
  transform.transform.translation.x = config.translation_xyz[0];
  transform.transform.translation.y = config.translation_xyz[1];
  transform.transform.translation.z = config.translation_xyz[2];

  const auto angles_rad =
    convert_angles_to_rad(config.rotation_angles, rotation_config.angle_unit);
  const auto rotation_matrix =
    euler_to_matrix(angles_rad, rotation_config.rotation_order);

  tf2::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);
  quaternion.normalize();

  if (!std::isfinite(quaternion.x()) ||
    !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) ||
    !std::isfinite(quaternion.w()))
  {
    throw std::invalid_argument(
            "Rotation for transform " + config.parent_frame + " -> " +
            config.child_frame + " produced a non-finite quaternion.");
  }

  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}

tf2::Vector3 vector_from_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  return tf2::Vector3(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
}

bool solve_3x3(
  std::array<std::array<double, 3>, 3> matrix,
  std::array<double, 3> rhs,
  std::array<double, 3> & solution)
{
  for (std::size_t column = 0; column < 3; ++column) {
    std::size_t pivot = column;
    double pivot_abs = std::abs(matrix[column][column]);
    for (std::size_t row = column + 1; row < 3; ++row) {
      const double row_abs = std::abs(matrix[row][column]);
      if (row_abs > pivot_abs) {
        pivot = row;
        pivot_abs = row_abs;
      }
    }

    if (pivot_abs < kEpsilon) {
      return false;
    }

    if (pivot != column) {
      std::swap(matrix[pivot], matrix[column]);
      std::swap(rhs[pivot], rhs[column]);
    }

    const double divisor = matrix[column][column];
    for (std::size_t item = column; item < 3; ++item) {
      matrix[column][item] /= divisor;
    }
    rhs[column] /= divisor;

    for (std::size_t row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      for (std::size_t item = column; item < 3; ++item) {
        matrix[row][item] -= factor * matrix[column][item];
      }
      rhs[row] -= factor * rhs[column];
    }
  }

  solution = rhs;
  return true;
}

tf2::Vector3 fit_ground_normal(const std::vector<tf2::Vector3> & points)
{
  std::array<std::array<double, 3>, 3> normal_matrix = {{
    {{0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}},
    {{0.0, 0.0, 0.0}},
  }};
  std::array<double, 3> rhs = {{0.0, 0.0, 0.0}};

  for (const auto & point : points) {
    const std::array<double, 3> row = {{point.x(), point.y(), 1.0}};
    for (std::size_t r = 0; r < 3; ++r) {
      rhs[r] += row[r] * point.z();
      for (std::size_t c = 0; c < 3; ++c) {
        normal_matrix[r][c] += row[r] * row[c];
      }
    }
  }

  std::array<double, 3> plane;
  if (!solve_3x3(normal_matrix, rhs, plane)) {
    throw std::invalid_argument(
            "Foot points are degenerate; cannot fit ground plane z = ax + by + c.");
  }

  tf2::Vector3 normal(-plane[0], -plane[1], 1.0);
  if (normal.length2() < kEpsilon) {
    throw std::invalid_argument("Ground plane normal is degenerate.");
  }
  normal.normalize();

  const tf2::Vector3 base_z(0.0, 0.0, 1.0);
  if (normal.dot(base_z) < 0.0) {
    normal = -normal;
  }
  return normal;
}

tf2::Vector3 centroid(const std::vector<tf2::Vector3> & points)
{
  tf2::Vector3 center(0.0, 0.0, 0.0);
  for (const auto & point : points) {
    center += point;
  }
  return center / static_cast<double>(points.size());
}

geometry_msgs::msg::TransformStamped make_ground_odom_transform(
  const rclcpp::Time & stamp,
  const GroundOdomConfig & config,
  const std::vector<tf2::Vector3> & foot_points_in_base)
{
  const tf2::Vector3 ground_z = fit_ground_normal(foot_points_in_base);
  const tf2::Vector3 ground_origin_in_base =
    centroid(foot_points_in_base) - config.foot_radius * ground_z;

  tf2::Vector3 ground_x = tf2::Vector3(1.0, 0.0, 0.0) -
    ground_z * tf2::Vector3(1.0, 0.0, 0.0).dot(ground_z);
  if (ground_x.length2() < kEpsilon) {
    ground_x = tf2::Vector3(0.0, 1.0, 0.0) -
      ground_z * tf2::Vector3(0.0, 1.0, 0.0).dot(ground_z);
  }
  if (ground_x.length2() < kEpsilon) {
    throw std::invalid_argument("Cannot project base x/y axis onto the ground plane.");
  }
  ground_x.normalize();

  tf2::Vector3 ground_y = ground_z.cross(ground_x);
  if (ground_y.length2() < kEpsilon) {
    throw std::invalid_argument("Ground odom y axis is degenerate.");
  }
  ground_y.normalize();
  ground_x = ground_y.cross(ground_z);
  ground_x.normalize();

  const tf2::Matrix3x3 base_rotation_ground(
    ground_x.x(), ground_y.x(), ground_z.x(),
    ground_x.y(), ground_y.y(), ground_z.y(),
    ground_x.z(), ground_y.z(), ground_z.z());
  tf2::Transform base_to_ground(base_rotation_ground, ground_origin_in_base);
  tf2::Transform ground_to_base = base_to_ground.inverse();

  tf2::Quaternion quaternion = ground_to_base.getRotation();
  quaternion.normalize();
  const tf2::Vector3 translation = ground_to_base.getOrigin();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = config.parent_frame;
  transform.child_frame_id = config.child_frame;
  transform.transform.translation.x = translation.x();
  transform.transform.translation.y = translation.y();
  transform.transform.translation.z = translation.z();
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}

}  // namespace

class GroundOdomTfNode : public rclcpp::Node
{
public:
  GroundOdomTfNode()
  : Node("ground_odom_tf_node"),
    broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(this)),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto rotation_config = load_rotation_config(*this);
    const auto ground_odom_config = load_ground_odom_config(*this);
    const auto initial_base_to_tracking_origin =
      load_transform_config(*this, "initial_base_to_tracking_origin");
    const auto tracking_body_to_base =
      load_transform_config(*this, "tracking_body_to_base");

    const auto foot_points = lookup_foot_points(ground_odom_config);
    const auto stamp = this->get_clock()->now();

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(3);
    transforms.push_back(
      make_ground_odom_transform(stamp, ground_odom_config, foot_points));
    transforms.push_back(
      to_transform_stamped(stamp, initial_base_to_tracking_origin, rotation_config));
    transforms.push_back(
      to_transform_stamped(stamp, tracking_body_to_base, rotation_config));

    broadcaster_->sendTransform(transforms);

    RCLCPP_INFO(
      get_logger(),
      "Published static TF %s -> %s, %s -> %s, and %s -> %s",
      transforms[0].header.frame_id.c_str(),
      transforms[0].child_frame_id.c_str(),
      transforms[1].header.frame_id.c_str(),
      transforms[1].child_frame_id.c_str(),
      transforms[2].header.frame_id.c_str(),
      transforms[2].child_frame_id.c_str());
  }

private:
  std::vector<tf2::Vector3> lookup_foot_points(const GroundOdomConfig & config)
  {
    std::vector<tf2::Vector3> foot_points;
    foot_points.reserve(config.foot_frames.size());

    const rclcpp::Duration timeout =
      rclcpp::Duration::from_seconds(config.tf_timeout_sec);
    const rclcpp::Time latest_time(0, 0, RCL_ROS_TIME);
    for (const auto & foot_frame : config.foot_frames) {
      std::string error;
      if (!tf_buffer_.canTransform(
          config.base_frame, foot_frame, latest_time, timeout, &error))
      {
        throw std::runtime_error(
                "Timed out waiting for TF " + config.base_frame + " -> " +
                foot_frame + ": " + error);
      }

      const auto transform =
        tf_buffer_.lookupTransform(config.base_frame, foot_frame, latest_time);
      foot_points.push_back(vector_from_transform(transform));
      RCLCPP_INFO(
        get_logger(),
        "Using foot point %s in %s: [%.6f, %.6f, %.6f]",
        foot_frame.c_str(),
        config.base_frame.c_str(),
        foot_points.back().x(),
        foot_points.back().y(),
        foot_points.back().z());
    }

    return foot_points;
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<GroundOdomTfNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ground_odom_tf_node"),
      "Failed to start ground_odom_tf_node: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
