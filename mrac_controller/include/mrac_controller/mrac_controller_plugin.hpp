/*
 * mrac_controller_plugin.hpp — robust direct MRAC as a ros2_control controller.
 *
 * Loaded by the controller manager (gz_ros2_control in simulation) under the
 * name `mrac_arm_controller`. It claims the effort command interfaces of the
 * four arm joints and their position/velocity state interfaces, and serves
 *   ~/follow_joint_trajectory   (control_msgs/action/FollowJointTrajectory)
 *   ~/save_weights, ~/load_weights   (std_srvs/srv/Trigger)
 * The maths is in mrac_core.hpp; this file is only the ROS wiring.
 */
#pragma once

#include "mrac_controller/mrac_core.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace mrac_controller
{

class MracController : public controller_interface::ControllerInterface
{
public:
  using FollowJT      = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJT>;
  using Float64Array  = std_msgs::msg::Float64MultiArray;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  enum Mode : int { IDLE = 0, EXECUTING = 1, HOLD = 2 };
  enum DoneCode : int { DONE_NONE = 0, DONE_SUCCESS = 1, DONE_TOLERANCE = 2 };

  // What the action side hands to the control loop.
  struct Command
  {
    std::uint64_t id = 0;
    bool hold = false;                              // true: stop and hold the current pose
    trajectory_msgs::msg::JointTrajectory traj;
    std::array<std::size_t, N_JOINTS> col{};        // trajectory column of each joint
    double t_end = 0.0;
  };

  bool readParameters();
  bool readState(Vec4 &q, Vec4 &qd);
  void writeEffort(const Vec4 &tau);
  void publishDiagnostics(const MracOutput &out, const Vec4 &q, const Vec4 &q_d);
  std::string weightsPath() const;

  // Action server (non-real-time threads)
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJT::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandleFJT>);
  void handleAccepted(std::shared_ptr<GoalHandleFJT> gh);
  void goalMonitor();
  void sendHoldCommand();

  // ── Parameters ──
  std::vector<std::string> joints_;
  MracParams params_;
  double goal_tolerance_ = 0.05;
  double goal_time_tolerance_ = 5.0;
  int diag_decimation_ = 1;
  std::string weights_file_;
  bool load_weights_on_activate_ = false;

  // ── Control loop state (touched only by update / on_activate) ──
  MracCore core_;
  std::mutex core_mx_;                 // update() try-locks; services lock briefly
  int mode_ = IDLE;
  std::shared_ptr<Command> active_;
  std::uint64_t active_id_ = 0;
  double t_traj_ = 0.0;
  Vec4 q_hold_ = Vec4::Zero();
  Vec4 q_goal_ = Vec4::Zero();
  Vec4 q_last_ = Vec4::Zero(), qd_last_ = Vec4::Zero(), tau_last_ = Vec4::Zero();
  int diag_count_ = 0;

  // ── Hand-off between the action side and the control loop ──
  realtime_tools::RealtimeBuffer<std::shared_ptr<Command>> rt_cmd_;
  std::atomic<std::uint64_t> done_id_{0};
  std::atomic<int> done_code_{DONE_NONE};
  std::atomic<bool> robust_enabled_{true};
  std::atomic<bool> active_flag_{false};

  // ── Action side state ──
  std::mutex goal_mx_;
  std::shared_ptr<GoalHandleFJT> current_gh_;
  std::uint64_t current_id_ = 0;
  std::uint64_t next_id_ = 0;

  // ── ROS interfaces ──
  rclcpp_action::Server<FollowJT>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_, load_srv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robust_sub_;
  rclcpp::Publisher<Float64Array>::SharedPtr tau_pub_, khat_pub_, w_pub_, err_pub_, traj_err_pub_,
      uad_pub_, ref_pub_, lyap_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
  Float64Array tau_msg_, khat_msg_, w_msg_, err_msg_, traj_err_msg_, uad_msg_, ref_msg_, lyap_msg_;
};

}  // namespace mrac_controller
