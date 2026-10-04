// Centroidal Moment Pivot estimation (Sec. IV of Raza et al. 2021).
//
//   p_f = (p_0 - c_z) / f_z * f + c          (Eq. 20, planar ground)
//   f   = m (c_dd - g)                        (Eq. 19)
//
// c is the whole-body CoM, computed from the URDF inertials, the joint states
// and the ground-truth pose of base_link. c_dd is obtained with a second-order
// state-variable filter. The CMP deviation from the ground projection of the
// CoM, expressed in the robot heading frame, is the balance metric; its time
// integral (in m.s) is the error reported in Table 2.
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "urdf_parser/urdf_parser.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace wheel_legged_arm_balance
{

class CmpEstimatorNode : public rclcpp::Node
{
public:
  CmpEstimatorNode()
  : Node("cmp_estimator")
  {
    ground_z_ = declare_parameter<double>("ground_height", 0.0);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    cutoff_hz_ = declare_parameter<double>("filter_cutoff_hz", 10.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    marker_rate_ = declare_parameter<double>("marker_rate", 30.0);
    path_rate_ = declare_parameter<double>("path_rate", 10.0);
    max_path_len_ = declare_parameter<int>("max_path_length", 5000);
    warmup_ = declare_parameter<double>("warmup_time", 0.5);
    if (declare_parameter<bool>("publish_tf", true)) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    desc_sub_ = create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr m) {loadModel(m->data);});
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr m) {
        for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
          q_[m->name[i]] = m->position[i];
        }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "ground_truth/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr m) {onOdom(*m);});

    com_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/com", 10);
    cmp_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/cmp", 10);
    err_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("~/cmp_error", 10);
    int_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("~/cmp_error_integral", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    cmp_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/cmp_path", 10);
    base_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/base_path", 10);
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        int_x_ = int_y_ = 0.0;
        cmp_path_.poses.clear();
        base_path_.poses.clear();
        res->success = true;
        res->message = "CMP error integral and paths reset";
      });
  }

private:
  struct Body
  {
    std::string link;
    double mass;
    Eigen::Vector3d cog;   // in link frame
  };

  void loadModel(const std::string & xml)
  {
    // urdfdom is used directly (not urdf::Model) to avoid the plugin loader
    // being unloaded before the model on shutdown
    model_ = urdf::parseURDF(xml);
    if (!model_) {
      RCLCPP_ERROR(get_logger(), "Could not parse robot_description");
      return;
    }
    bodies_.clear();
    total_mass_ = 0.0;
    std::vector<urdf::LinkSharedPtr> links;
    model_->getLinks(links);
    for (const auto & l : links) {
      if (l->inertial && l->inertial->mass > 0.0) {
        const auto & o = l->inertial->origin.position;
        bodies_.push_back({l->name, l->inertial->mass, Eigen::Vector3d(o.x, o.y, o.z)});
        total_mass_ += l->inertial->mass;
      }
    }
    root_ = model_->getRoot()->name;
    RCLCPP_INFO(get_logger(), "CMP estimator: %zu bodies, total mass %.3f kg, root '%s'",
      bodies_.size(), total_mass_, root_.c_str());
  }

  static Eigen::Isometry3d toIso(const urdf::Pose & p)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() << p.position.x, p.position.y, p.position.z;
    T.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z)
      .toRotationMatrix();
    return T;
  }

  /// Pose of a link in the root frame (recursive over the URDF tree).
  Eigen::Isometry3d linkPose(const std::string & name)
  {
    auto link = model_->getLink(name);
    if (!link || name == root_) {
      return Eigen::Isometry3d::Identity();
    }
    const auto & j = link->parent_joint;
    Eigen::Isometry3d T = linkPose(link->getParent()->name) *
      toIso(j->parent_to_joint_origin_transform);
    if (j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::CONTINUOUS) {
      const double q = q_.count(j->name) ? q_[j->name] : 0.0;
      const Eigen::Vector3d ax(j->axis.x, j->axis.y, j->axis.z);
      T.rotate(Eigen::AngleAxisd(q, ax.normalized()));
    } else if (j->type == urdf::Joint::PRISMATIC) {
      const double q = q_.count(j->name) ? q_[j->name] : 0.0;
      T.translate(q * Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z));
    }
    return T;
  }

  void onOdom(const nav_msgs::msg::Odometry & m)
  {
    const auto & P = m.pose.pose;
    if (tf_) {
      // world -> base_link from the ground truth (for RViz)
      const rclcpp::Time stamp(m.header.stamp);
      if (stamp != last_tf_stamp_) {
        last_tf_stamp_ = stamp;
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = m.header.stamp;
        tf.header.frame_id = world_frame_;
        tf.child_frame_id = m.child_frame_id.empty() ? "base_link" : m.child_frame_id;
        tf.transform.translation.x = P.position.x;
        tf.transform.translation.y = P.position.y;
        tf.transform.translation.z = P.position.z;
        tf.transform.rotation = P.orientation;
        tf_->sendTransform(tf);
      }
    }
    if (!model_ || total_mass_ <= 0.0) {
      return;
    }
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.translation() << P.position.x, P.position.y, P.position.z;
    const Eigen::Quaterniond q_wb(P.orientation.w, P.orientation.x, P.orientation.y,
      P.orientation.z);
    T_wb.linear() = q_wb.toRotationMatrix();

    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const auto & b : bodies_) {
      c += b.mass * (T_wb * linkPose(b.link) * b.cog);
    }
    c /= total_mass_;

    const rclcpp::Time t(m.header.stamp);
    if (!initialised_) {
      t0_ = t;
      cf_ = c;
      cv_.setZero();
      ca_.setZero();
      last_t_ = t;
      initialised_ = true;
      return;
    }
    const double dt = (t - last_t_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_t_ = t;
    // state-variable filter: H(s) = w^2 / (s + w)^2 ; derivatives are filtered too
    const double w = 2.0 * M_PI * cutoff_hz_;
    const int sub = std::max(1, static_cast<int>(std::ceil(dt / 0.0005)));
    const double h = dt / sub;
    for (int i = 0; i < sub; ++i) {
      ca_ = w * w * (c - cf_) - 2.0 * w * cv_;
      cv_ += ca_ * h;
      cf_ += cv_ * h;
    }
    if ((t - t0_).seconds() < warmup_) {
      return;   // let the derivative filter converge first
    }

    // Eq. (19) and (20)
    const Eigen::Vector3d g(0.0, 0.0, -gravity_);
    const Eigen::Vector3d f = total_mass_ * (ca_ - g);
    Eigen::Vector3d cmp = c;
    if (std::abs(f.z()) > 1e-6) {
      cmp = (ground_z_ - c.z()) / f.z() * f + c;
    }

    // deviation from the CoM ground projection, in the heading frame
    const Eigen::Vector3d xb = T_wb.linear().col(0);
    const double yaw = std::atan2(xb.y(), xb.x());
    const double dx = cmp.x() - c.x(), dy = cmp.y() - c.y();
    const double ex = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ey = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    int_x_ += std::abs(ex) * dt;
    int_y_ += std::abs(ey) * dt;

    publishPoint(com_pub_, m.header.stamp, c);
    publishPoint(cmp_pub_, m.header.stamp, cmp);
    geometry_msgs::msg::Vector3Stamped e;
    e.header.stamp = m.header.stamp;
    e.header.frame_id = "heading";
    e.vector.x = ex;
    e.vector.y = ey;
    e.vector.z = std::hypot(ex, ey);
    err_pub_->publish(e);
    e.vector.x = int_x_;
    e.vector.y = int_y_;
    e.vector.z = 0.0;
    int_pub_->publish(e);

    if ((t - last_marker_).seconds() >= 1.0 / marker_rate_) {
      last_marker_ = t;
      publishMarkers(m.header.stamp, c, cmp);
    }
    if ((t - last_path_).seconds() >= 1.0 / path_rate_) {
      last_path_ = t;
      appendPath(cmp_path_, m.header.stamp, Eigen::Vector3d(cmp.x(), cmp.y(), ground_z_ + 0.002));
      appendPath(base_path_, m.header.stamp,
        Eigen::Vector3d(P.position.x, P.position.y, ground_z_ + 0.001));
      cmp_path_pub_->publish(cmp_path_);
      base_path_pub_->publish(base_path_);
    }
  }

  void publishPoint(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & pub,
    const builtin_interfaces::msg::Time & stamp, const Eigen::Vector3d & p)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;
    msg.point.x = p.x();
    msg.point.y = p.y();
    msg.point.z = p.z();
    pub->publish(msg);
  }

  void appendPath(nav_msgs::msg::Path & path, const builtin_interfaces::msg::Time & stamp,
    const Eigen::Vector3d & p)
  {
    path.header.stamp = stamp;
    path.header.frame_id = world_frame_;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
    if (static_cast<int>(path.poses.size()) > max_path_len_) {
      path.poses.erase(path.poses.begin());
    }
  }

  void publishMarkers(
    const builtin_interfaces::msg::Time & stamp, const Eigen::Vector3d & c,
    const Eigen::Vector3d & cmp)
  {
    visualization_msgs::msg::MarkerArray arr;
    auto sphere = [&](int id, const Eigen::Vector3d & p, float r, float g, float b, double s) {
        visualization_msgs::msg::Marker mk;
        mk.header.stamp = stamp;
        mk.header.frame_id = world_frame_;
        mk.ns = "cmp";
        mk.id = id;
        mk.type = visualization_msgs::msg::Marker::SPHERE;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose.position.x = p.x();
        mk.pose.position.y = p.y();
        mk.pose.position.z = p.z();
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = mk.scale.z = s;
        mk.color.r = r;
        mk.color.g = g;
        mk.color.b = b;
        mk.color.a = 0.9f;
        return mk;
      };
    // as in Fig. 7: blue = total CoM, pink = CMP
    arr.markers.push_back(sphere(0, c, 0.1f, 0.3f, 1.0f, 0.06));
    arr.markers.push_back(sphere(1, cmp, 1.0f, 0.4f, 0.8f, 0.06));
    arr.markers.push_back(sphere(2, Eigen::Vector3d(c.x(), c.y(), ground_z_), 0.1f, 0.3f, 1.0f,
      0.03));
    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = world_frame_;
    line.ns = "cmp";
    line.id = 3;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.scale.x = 0.008;
    line.color.r = 1.0f;
    line.color.g = 0.4f;
    line.color.b = 0.8f;
    line.color.a = 0.9f;
    line.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point a, b;
    a.x = c.x();
    a.y = c.y();
    a.z = c.z();
    b.x = cmp.x();
    b.y = cmp.y();
    b.z = cmp.z();
    line.points = {a, b};
    arr.markers.push_back(line);
    marker_pub_->publish(arr);
  }

  double warmup_{0.5};
  rclcpp::Time t0_{0, 0, RCL_ROS_TIME};
  double ground_z_, gravity_, cutoff_hz_, marker_rate_, path_rate_;
  int max_path_len_;
  std::string world_frame_, root_;
  urdf::ModelInterfaceSharedPtr model_;
  std::vector<Body> bodies_;
  double total_mass_{0.0};
  std::map<std::string, double> q_;

  bool initialised_{false};
  Eigen::Vector3d cf_, cv_, ca_;
  double int_x_{0.0}, int_y_{0.0};
  rclcpp::Time last_t_{0, 0, RCL_ROS_TIME}, last_marker_{0, 0, RCL_ROS_TIME},
  last_path_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path cmp_path_, base_path_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_pub_, cmp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr err_pub_, int_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr cmp_path_pub_, base_path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace wheel_legged_arm_balance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_legged_arm_balance::CmpEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
