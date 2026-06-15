// Copyright 2026 OMX IK-ACT Hybrid Control contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotis_interfaces/msg/move_l.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "common/type_define.hpp"
#include "controllers/open_manipulator/open_manipulator_movel_controller.hpp"
#include "kinematics/kinematics_solver.hpp"

namespace omx_cyclo_safe_controller
{

using cyclo_motion_controller::common::Vector6d;
using cyclo_motion_controller::common::collision_checker::MinDistResult;
using cyclo_motion_controller::controllers::OpenManipulatorMoveLController;
using cyclo_motion_controller::kinematics::KinematicsSolver;

class StartupRelaxedKinematicsSolver : public KinematicsSolver
{
public:
  StartupRelaxedKinematicsSolver(
    const std::string & urdf_path,
    const std::string & srdf_path,
    std::string sphere_geometry_name,
    const double sphere_radius,
    const double restore_distance)
  : KinematicsSolver(urdf_path, srdf_path),
    sphere_geometry_name_(std::move(sphere_geometry_name)),
    sphere_radius_(sphere_radius),
    restore_distance_(restore_distance)
  {
    if (sphere_radius_ < 0.0) {
      throw std::invalid_argument("startup_sphere_radius must not be negative");
    }
    if (restore_distance_ < 0.0) {
      throw std::invalid_argument("startup_restore_distance must not be negative");
    }

    const auto geometry_it = std::find_if(
      geom_model_.geometryObjects.begin(), geom_model_.geometryObjects.end(),
      [this](const auto & geometry) {
        return geometry.name == sphere_geometry_name_;
      });
    if (geometry_it == geom_model_.geometryObjects.end()) {
      throw std::runtime_error(
              "Startup sphere geometry '" + sphere_geometry_name_ + "' was not found");
    }
  }

  void enableStartupRelaxation()
  {
    startup_relaxation_active_ = true;
  }

  bool startupRelaxationActive() const
  {
    return startup_relaxation_active_;
  }

  double rawMinimumRelaxedDistance() const
  {
    return raw_minimum_relaxed_distance_;
  }

  std::vector<MinDistResult> getCollisionPairDistances(
    const bool & with_grad,
    const bool & with_graddot,
    const bool verbose) override
  {
    auto results = KinematicsSolver::getCollisionPairDistances(with_grad, with_graddot, verbose);
    raw_minimum_relaxed_distance_ = std::numeric_limits<double>::infinity();

    const int pair_count = std::min<int>(results.size(), geom_model_.collisionPairs.size());
    for (int i = 0; i < pair_count; ++i) {
      const auto & pair = geom_model_.collisionPairs[static_cast<std::size_t>(i)];
      if (pairUsesStartupSphere(pair)) {
        raw_minimum_relaxed_distance_ =
          std::min(raw_minimum_relaxed_distance_, results[static_cast<std::size_t>(i)].distance);
      }
    }

    if (
      startup_relaxation_active_ &&
      std::isfinite(raw_minimum_relaxed_distance_) &&
      raw_minimum_relaxed_distance_ >= restore_distance_)
    {
      startup_relaxation_active_ = false;
    }

    if (startup_relaxation_active_) {
      for (int i = 0; i < pair_count; ++i) {
        const auto & pair = geom_model_.collisionPairs[static_cast<std::size_t>(i)];
        if (pairUsesStartupSphere(pair)) {
          // Distance to a sphere is distance to its center minus its radius.
          // Adding the radius makes the startup collision sphere a radius-zero point.
          results[static_cast<std::size_t>(i)].distance += sphere_radius_;
        }
      }
    }

    return results;
  }

private:
  bool pairUsesStartupSphere(const pinocchio::CollisionPair & pair) const
  {
    return
      geom_model_.geometryObjects[pair.first].name == sphere_geometry_name_ ||
      geom_model_.geometryObjects[pair.second].name == sphere_geometry_name_;
  }

  std::string sphere_geometry_name_;
  double sphere_radius_;
  double restore_distance_;
  bool startup_relaxation_active_ {false};
  double raw_minimum_relaxed_distance_ {std::numeric_limits<double>::infinity()};
};

class OmxSafeMoveLControllerNode : public rclcpp::Node
{
public:
  OmxSafeMoveLControllerNode()
  : Node("omx_movel_controller"),
    motion_start_time_(this->now()),
    last_joint_state_time_(this->now())
  {
    declareParameters();
    validateParameters();

    if (urdf_path_.empty()) {
      throw std::runtime_error("urdf_path must be provided");
    }

    kinematics_solver_ = std::make_shared<StartupRelaxedKinematicsSolver>(
      urdf_path_, srdf_path_, startup_sphere_geometry_, startup_sphere_radius_,
      startup_restore_distance_);
    qp_controller_ = std::make_shared<OpenManipulatorMoveLController>(
      kinematics_solver_, controlled_link_, time_step_);
    qp_controller_->setControllerParams(
      slack_penalty_, cbf_alpha_, collision_buffer_, collision_safe_distance_);

    model_joint_names_ = kinematics_solver_->getJointNames();
    q_.setZero(kinematics_solver_->getDof());
    qdot_.setZero(kinematics_solver_->getDof());
    q_commanded_.setZero(kinematics_solver_->getDof());

    initializeStartupLock();
    initializeRosInterfaces();

    const int timer_period_ms =
      std::max(1, static_cast<int>(std::round(1000.0 / std::max(1.0, control_frequency_))));
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms),
      std::bind(&OmxSafeMoveLControllerNode::controlLoopCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Safe OMX MoveL ready. No trajectory is published before the first MoveL command.");
  }

private:
  void declareParameters()
  {
    control_frequency_ = declare_parameter("control_frequency", 100.0);
    time_step_ = declare_parameter("time_step", 0.01);
    trajectory_time_ = declare_parameter("trajectory_time", 0.0);
    kp_position_ = declare_parameter("kp_position", 50.0);
    kp_orientation_ = declare_parameter("kp_orientation", 50.0);
    weight_task_position_ = declare_parameter("weight_task_position", 10.0);
    weight_task_orientation_ = declare_parameter("weight_task_orientation", 1.0);
    weight_damping_ = declare_parameter("weight_damping", 0.001);
    slack_penalty_ = declare_parameter("slack_penalty", 1000.0);
    cbf_alpha_ = declare_parameter("cbf_alpha", 5.0);
    collision_buffer_ = declare_parameter("collision_buffer", 0.01);
    collision_safe_distance_ = declare_parameter("collision_safe_distance", 0.005);
    joint_state_timeout_ = declare_parameter("joint_state_timeout", 0.5);

    urdf_path_ = declare_parameter("urdf_path", std::string(""));
    srdf_path_ = declare_parameter("srdf_path", std::string(""));
    base_frame_ = declare_parameter("base_frame", std::string("link0"));
    controlled_link_ = declare_parameter("controlled_link", std::string("end_effector_link"));
    joint_states_topic_ = declare_parameter("joint_states_topic", std::string("/joint_states"));
    joint_command_topic_ =
      declare_parameter("joint_command_topic", std::string("/leader/joint_trajectory"));
    movel_topic_ = declare_parameter("movel_topic", std::string("~/movel"));
    ee_pose_topic_ = declare_parameter("ee_pose_topic", std::string("~/current_pose"));
    controller_error_topic_ =
      declare_parameter("controller_error_topic", std::string("~/controller_error"));
    startup_relaxation_topic_ =
      declare_parameter("startup_relaxation_topic", std::string("~/startup_relaxation_active"));

    startup_relaxation_enabled_ = declare_parameter("startup_relaxation_enabled", true);
    startup_sphere_geometry_ =
      declare_parameter("startup_sphere_geometry", std::string("link5_0"));
    startup_sphere_radius_ = declare_parameter("startup_sphere_radius", 0.05);
    startup_restore_distance_ = declare_parameter("startup_restore_distance", 0.007);
    startup_home_positions_ =
      declare_parameter<std::vector<double>>(
      "startup_home_positions", {0.0, -1.57, 1.57, 1.57, 0.0});
    startup_home_tolerance_ = declare_parameter("startup_home_tolerance", 0.15);
    startup_require_closed_gripper_ = declare_parameter("startup_require_closed_gripper", true);
    startup_gripper_joint_name_ =
      declare_parameter("startup_gripper_joint_name", std::string("gripper_joint_1"));
    startup_gripper_closed_position_ =
      declare_parameter("startup_gripper_closed_position", 0.0);
    startup_gripper_tolerance_ = declare_parameter("startup_gripper_tolerance", 0.15);
    startup_lock_joint_name_ = declare_parameter("startup_lock_joint_name", std::string("joint5"));
  }

  void validateParameters() const
  {
    if (startup_restore_distance_ < collision_safe_distance_) {
      throw std::runtime_error(
              "startup_restore_distance must be greater than or equal to collision_safe_distance");
    }
    if (startup_home_tolerance_ < 0.0 || startup_gripper_tolerance_ < 0.0) {
      throw std::runtime_error("Startup tolerances must not be negative");
    }
  }

  void initializeStartupLock()
  {
    const auto bounds = kinematics_solver_->getJointVelocityLimit();
    const auto lock_it =
      std::find(model_joint_names_.begin(), model_joint_names_.end(), startup_lock_joint_name_);
    if (lock_it == model_joint_names_.end()) {
      if (!startup_lock_joint_name_.empty()) {
        RCLCPP_WARN(
          get_logger(), "Startup lock joint '%s' was not found.", startup_lock_joint_name_.c_str());
      }
      return;
    }

    startup_lock_joint_index_ =
      static_cast<int>(std::distance(model_joint_names_.begin(), lock_it));
    startup_lock_joint_lower_ = bounds.first[startup_lock_joint_index_];
    startup_lock_joint_upper_ = bounds.second[startup_lock_joint_index_];
  }

  void initializeRosInterfaces()
  {
    joint_command_pub_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_command_topic_, 10);
    ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(ee_pose_topic_, 10);
    controller_error_pub_ = create_publisher<std_msgs::msg::String>(controller_error_topic_, 10);

    const auto status_qos = rclcpp::QoS(1).transient_local().reliable();
    startup_relaxation_pub_ =
      create_publisher<std_msgs::msg::Bool>(startup_relaxation_topic_, status_qos);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10,
      std::bind(&OmxSafeMoveLControllerNode::jointStateCallback, this, std::placeholders::_1));
    movel_sub_ = create_subscription<robotis_interfaces::msg::MoveL>(
      movel_topic_, 10,
      std::bind(&OmxSafeMoveLControllerNode::moveLCallback, this, std::placeholders::_1));

    publishStartupRelaxationStatus(false);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (joint_index_map_.empty()) {
      for (std::size_t i = 0; i < msg->name.size(); ++i) {
        joint_index_map_[msg->name[i]] = static_cast<int>(i);
      }
    }

    extractJointStates(*msg);
    startup_gripper_guard_ok_ = gripperStartsClosed(*msg);
    last_joint_state_time_ = now();
    joint_state_received_ = true;
    joint_state_timeout_active_ = false;

    if (!commanded_state_initialized_) {
      syncCommandStateToFeedback();
      commanded_state_initialized_ = true;
      initializeStartupRelaxation(*msg);
    }

    if (kinematics_solver_->startupRelaxationActive() && !startup_gripper_guard_ok_) {
      movel_trajectory_active_ = false;
      movel_target_initialized_ = false;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Gripper opened while startup relaxation is active; arm trajectory output is stopped.");
    }
  }

  void extractJointStates(const sensor_msgs::msg::JointState & msg)
  {
    q_.setZero(kinematics_solver_->getDof());
    qdot_.setZero(kinematics_solver_->getDof());

    for (int i = 0; i < static_cast<int>(model_joint_names_.size()); ++i) {
      const auto state_it = joint_index_map_.find(model_joint_names_[static_cast<std::size_t>(i)]);
      if (state_it == joint_index_map_.end()) {
        continue;
      }
      const int state_index = state_it->second;
      if (state_index < static_cast<int>(msg.position.size())) {
        q_[i] = msg.position[static_cast<std::size_t>(state_index)];
      }
      if (state_index < static_cast<int>(msg.velocity.size())) {
        qdot_[i] = msg.velocity[static_cast<std::size_t>(state_index)];
      }
    }
  }

  bool startsAtConfiguredHome() const
  {
    if (startup_home_positions_.size() != static_cast<std::size_t>(q_.size())) {
      RCLCPP_ERROR(
        get_logger(), "startup_home_positions has %zu values, expected %ld.",
        startup_home_positions_.size(), static_cast<long>(q_.size()));
      return false;
    }

    for (int i = 0; i < q_.size(); ++i) {
      if (std::abs(q_[i] - startup_home_positions_[static_cast<std::size_t>(i)]) >
        startup_home_tolerance_)
      {
        return false;
      }
    }
    return true;
  }

  bool gripperStartsClosed(const sensor_msgs::msg::JointState & msg) const
  {
    if (!startup_require_closed_gripper_) {
      return true;
    }

    const auto gripper_it =
      std::find(msg.name.begin(), msg.name.end(), startup_gripper_joint_name_);
    if (gripper_it == msg.name.end()) {
      RCLCPP_ERROR_ONCE(
        get_logger(), "Required gripper joint '%s' is missing from joint states.",
        startup_gripper_joint_name_.c_str());
      return false;
    }

    const auto gripper_index = static_cast<std::size_t>(std::distance(msg.name.begin(), gripper_it));
    if (gripper_index >= msg.position.size()) {
      return false;
    }
    return std::abs(msg.position[gripper_index] - startup_gripper_closed_position_) <=
           startup_gripper_tolerance_;
  }

  void initializeStartupRelaxation(const sensor_msgs::msg::JointState & msg)
  {
    if (
      startup_relaxation_enabled_ && startsAtConfiguredHome() &&
      gripperStartsClosed(msg))
    {
      kinematics_solver_->enableStartupRelaxation();
      setStartupJointLock(true);
      publishStartupRelaxationStatus(true);
      RCLCPP_WARN(
        get_logger(),
        "Startup relaxation enabled: '%s' is treated as a radius-zero point until all related "
        "raw distances reach %.4f m.",
        startup_sphere_geometry_.c_str(), startup_restore_distance_);
      return;
    }

    publishStartupRelaxationStatus(false);
    RCLCPP_INFO(
      get_logger(),
      "Startup relaxation was not enabled because the initial state did not satisfy its guards.");
  }

  void setStartupJointLock(const bool locked)
  {
    if (startup_lock_joint_index_ < 0) {
      return;
    }
    if (locked) {
      kinematics_solver_->setJointVelocityBoundsByIndex(startup_lock_joint_index_, 0.0, 0.0);
    } else {
      kinematics_solver_->setJointVelocityBoundsByIndex(
        startup_lock_joint_index_, startup_lock_joint_lower_, startup_lock_joint_upper_);
    }
  }

  void handleStartupRelaxationTransition(const bool was_active)
  {
    if (was_active && !kinematics_solver_->startupRelaxationActive()) {
      setStartupJointLock(false);
      publishStartupRelaxationStatus(false);
      RCLCPP_WARN(
        get_logger(),
        "Startup relaxation permanently disabled. Raw sphere clearance is %.4f m; normal %.4f m "
        "collision safety is active.",
        kinematics_solver_->rawMinimumRelaxedDistance(), collision_safe_distance_);
    }
  }

  void moveLCallback(const robotis_interfaces::msg::MoveL::SharedPtr msg)
  {
    if (!msg || !joint_state_received_ || jointStateTimedOut()) {
      RCLCPP_WARN(get_logger(), "Ignoring MoveL command until fresh joint states are available.");
      return;
    }
    if (kinematics_solver_->startupRelaxationActive() && !startup_gripper_guard_ok_) {
      RCLCPP_ERROR(
        get_logger(), "Ignoring MoveL command: close the gripper while startup relaxation is active.");
      return;
    }

    syncCommandStateToFeedback();
    const double requested_duration = rclcpp::Duration(msg->time_from_start).seconds();
    active_motion_duration_ = requested_duration;
    motion_start_time_ = now();
    movel_goal_pose_ = poseMsgToEigen(msg->pose);
    movel_target_initialized_ = true;
    movel_trajectory_active_ = requested_duration > 0.0;

    RCLCPP_INFO(
      get_logger(), "Accepted MoveL command. Startup relaxation: %s.",
      kinematics_solver_->startupRelaxationActive() ? "active" : "inactive");
  }

  void controlLoopCallback()
  {
    if (!joint_state_received_ || !commanded_state_initialized_) {
      return;
    }
    if (jointStateTimedOut()) {
      if (!joint_state_timeout_active_) {
        joint_state_timeout_active_ = true;
        movel_trajectory_active_ = false;
        RCLCPP_WARN(get_logger(), "Joint states timed out; trajectory output stopped.");
      }
      return;
    }
    if (kinematics_solver_->startupRelaxationActive() && !startup_gripper_guard_ok_) {
      return;
    }

    try {
      if (!movel_target_initialized_) {
        kinematics_solver_->updateState(q_, qdot_);
        publishCurrentPose(kinematics_solver_->getPose(controlled_link_));
        const bool was_active = kinematics_solver_->startupRelaxationActive();
        kinematics_solver_->getCollisionPairDistances(false, false, false);
        handleStartupRelaxationTransition(was_active);
        return;
      }

      const Eigen::VectorXd q_feedback = q_commanded_;
      kinematics_solver_->updateState(q_feedback, qdot_);
      const Eigen::Affine3d current_pose = kinematics_solver_->getPose(controlled_link_);
      publishCurrentPose(current_pose);

      const double elapsed = (now() - motion_start_time_).seconds();
      const Vector6d desired_velocity = computeDesiredVelocity(current_pose, elapsed);
      Vector6d task_weight = Vector6d::Zero();
      task_weight.head<3>().setConstant(weight_task_position_);
      task_weight.tail<3>().setConstant(weight_task_orientation_);
      const Eigen::VectorXd damping_weight =
        Eigen::VectorXd::Ones(kinematics_solver_->getDof()) * weight_damping_;

      qp_controller_->setDesiredTaskVel(desired_velocity);
      qp_controller_->setWeights(task_weight, damping_weight);

      const bool was_active = kinematics_solver_->startupRelaxationActive();
      Eigen::VectorXd optimal_velocities;
      if (!qp_controller_->getOptJointVel(optimal_velocities)) {
        publishControllerError("Safe OMX MoveL QP solve failed");
        return;
      }
      handleStartupRelaxationTransition(was_active);

      q_commanded_ = q_feedback + optimal_velocities * time_step_;
      publishTrajectory(q_commanded_);
    } catch (const std::exception & error) {
      publishControllerError("Safe OMX MoveL loop error: " + std::string(error.what()));
    }
  }

  Vector6d computeDesiredVelocity(const Eigen::Affine3d & current_pose, const double elapsed)
  {
    Vector6d desired_velocity = Vector6d::Zero();
    Eigen::Affine3d reference_pose = movel_goal_pose_;
    Eigen::Vector3d feedforward_linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d feedforward_angular = Eigen::Vector3d::Zero();

    if (movel_trajectory_active_ && elapsed < active_motion_duration_) {
      reference_pose.translation() =
        cyclo_motion_controller::common::math_utils::cubicVector<3>(
        elapsed, 0.0, active_motion_duration_, movel_start_pose_.translation(),
        movel_goal_pose_.translation(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      reference_pose.linear() =
        cyclo_motion_controller::common::math_utils::rotationCubic(
        elapsed, 0.0, active_motion_duration_, movel_start_pose_.linear(),
        movel_goal_pose_.linear());
      feedforward_linear =
        cyclo_motion_controller::common::math_utils::cubicDotVector<3>(
        elapsed, 0.0, active_motion_duration_, movel_start_pose_.translation(),
        movel_goal_pose_.translation(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      feedforward_angular =
        cyclo_motion_controller::common::math_utils::rotationCubicDot(
        elapsed, 0.0, active_motion_duration_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        movel_start_pose_.linear(), movel_goal_pose_.linear());
    } else {
      movel_trajectory_active_ = false;
    }

    desired_velocity.head<3>() =
      feedforward_linear + kp_position_ * (reference_pose.translation() - current_pose.translation());
    const Eigen::Matrix3d rotation_error =
      reference_pose.linear() * current_pose.linear().transpose();
    const Eigen::AngleAxisd angle_axis_error(rotation_error);
    desired_velocity.tail<3>() =
      feedforward_angular +
      kp_orientation_ * angle_axis_error.axis() * angle_axis_error.angle();
    return desired_velocity;
  }

  void syncCommandStateToFeedback()
  {
    q_commanded_ = q_;
    kinematics_solver_->updateState(q_commanded_, qdot_);
    movel_start_pose_ = kinematics_solver_->getPose(controlled_link_);
    if (!movel_target_initialized_) {
      movel_goal_pose_ = movel_start_pose_;
    }
    movel_trajectory_active_ = false;
  }

  Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::PoseStamped & pose_msg) const
  {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() <<
      pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
    const Eigen::Quaterniond quaternion(
      pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z);
    if (quaternion.norm() <= std::numeric_limits<double>::epsilon()) {
      throw std::invalid_argument("MoveL orientation quaternion must not be zero");
    }
    pose.linear() = quaternion.normalized().toRotationMatrix();
    return pose;
  }

  bool jointStateTimedOut() const
  {
    return joint_state_received_ && (now() - last_joint_state_time_).seconds() > joint_state_timeout_;
  }

  void publishCurrentPose(const Eigen::Affine3d & pose) const
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = base_frame_;
    message.pose.position.x = pose.translation().x();
    message.pose.position.y = pose.translation().y();
    message.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond quaternion(pose.linear());
    message.pose.orientation.w = quaternion.w();
    message.pose.orientation.x = quaternion.x();
    message.pose.orientation.y = quaternion.y();
    message.pose.orientation.z = quaternion.z();
    ee_pose_pub_->publish(message);
  }

  void publishTrajectory(const Eigen::VectorXd & command) const
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.joint_names = model_joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);
    point.positions.assign(command.data(), command.data() + command.size());
    point.velocities.assign(static_cast<std::size_t>(command.size()), 0.0);
    message.points.push_back(point);
    joint_command_pub_->publish(message);
  }

  void publishControllerError(const std::string & error) const
  {
    std_msgs::msg::String message;
    message.data = error;
    controller_error_pub_->publish(message);
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
  }

  void publishStartupRelaxationStatus(const bool active) const
  {
    std_msgs::msg::Bool message;
    message.data = active;
    startup_relaxation_pub_->publish(message);
  }

  double control_frequency_;
  double time_step_;
  double trajectory_time_;
  double kp_position_;
  double kp_orientation_;
  double weight_task_position_;
  double weight_task_orientation_;
  double weight_damping_;
  double slack_penalty_;
  double cbf_alpha_;
  double collision_buffer_;
  double collision_safe_distance_;
  double joint_state_timeout_;

  std::string urdf_path_;
  std::string srdf_path_;
  std::string base_frame_;
  std::string controlled_link_;
  std::string joint_states_topic_;
  std::string joint_command_topic_;
  std::string movel_topic_;
  std::string ee_pose_topic_;
  std::string controller_error_topic_;
  std::string startup_relaxation_topic_;

  bool startup_relaxation_enabled_;
  std::string startup_sphere_geometry_;
  double startup_sphere_radius_;
  double startup_restore_distance_;
  std::vector<double> startup_home_positions_;
  double startup_home_tolerance_;
  bool startup_require_closed_gripper_;
  std::string startup_gripper_joint_name_;
  double startup_gripper_closed_position_;
  double startup_gripper_tolerance_;
  std::string startup_lock_joint_name_;
  int startup_lock_joint_index_ {-1};
  double startup_lock_joint_lower_ {0.0};
  double startup_lock_joint_upper_ {0.0};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr startup_relaxation_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<robotis_interfaces::msg::MoveL>::SharedPtr movel_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<StartupRelaxedKinematicsSolver> kinematics_solver_;
  std::shared_ptr<OpenManipulatorMoveLController> qp_controller_;
  std::vector<std::string> model_joint_names_;
  std::unordered_map<std::string, int> joint_index_map_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_commanded_;

  bool joint_state_received_ {false};
  bool commanded_state_initialized_ {false};
  bool movel_target_initialized_ {false};
  bool movel_trajectory_active_ {false};
  bool joint_state_timeout_active_ {false};
  bool startup_gripper_guard_ok_ {false};
  rclcpp::Time motion_start_time_;
  rclcpp::Time last_joint_state_time_;
  double active_motion_duration_ {0.0};
  Eigen::Affine3d movel_start_pose_ {Eigen::Affine3d::Identity()};
  Eigen::Affine3d movel_goal_pose_ {Eigen::Affine3d::Identity()};
};

}  // namespace omx_cyclo_safe_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<omx_cyclo_safe_controller::OmxSafeMoveLControllerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("omx_safe_movel_controller"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
