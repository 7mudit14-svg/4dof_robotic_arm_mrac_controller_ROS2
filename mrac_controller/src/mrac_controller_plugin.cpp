/*
 * mrac_controller_plugin.cpp — ROS wiring of the robust direct MRAC controller.
 * See mrac_core.hpp for the control law and adaptation laws.
 *
 * Modes
 *   HOLD       r = q_hold (constant). K̂x and Ŵ adapt, K̂r is frozen.
 *   EXECUTING  r follows the Hermite-sampled trajectory, pre-compensated so
 *              the reference model tracks it without lag. All laws adapt.
 *
 * Topics (absolute names, unchanged from the standalone node where possible)
 *   /mrac/tau         [τ0..τ3]                      commanded torque
 *   /mrac/K_hat       [Kp0,Kd0,Kr0, Kp1,Kd1,Kr1, …] each joint's own gains
 *   /mrac/W_hat       [Ŵ[:,0], Ŵ[:,1], …]           N_BASIS values per joint
 *   /mrac/error       [e0,ė0, e1,ė1, …]             plant − reference model
 *   /mrac/traj_error  [q − q_d]                     plant − planned trajectory
 *   /mrac/u_ad        [−Ŵᵀα per joint]
 *   /mrac/ref_model   [q_m0,q̇_m0, …]
 *   /mrac/lyapunov_V  [eᵀPe per joint]              tracking-error energy
 *   /mrac/mode        Int32: 0 idle, 1 executing, 2 hold
 *   /mrac/robust_enable  (subscribed) std_msgs/Bool — runtime toggle of Ŵ
 */
#include "mrac_controller/mrac_controller_plugin.hpp"

#include "mrac_controller/trajectory_sampler.hpp"
#include "mrac_controller/weights_io.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>

namespace mrac_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

// ════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════════════════
CallbackReturn MracController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3", "joint4"});
    auto_declare<double>("wn", 3.0);
    auto_declare<double>("zeta", 0.76);
    auto_declare<double>("sign_b", 1.0);
    auto_declare<bool>("decentralized", true);
    auto_declare<bool>("robust_enabled", true);
    auto_declare<std::vector<double>>("gamma_Kx", {50.0, 50.0, 50.0, 50.0});
    auto_declare<std::vector<double>>("gamma_Kr", {50.0, 50.0, 50.0, 50.0});
    auto_declare<std::vector<double>>("gamma_W", {50.0, 50.0, 50.0, 50.0});
    auto_declare<double>("sigma", 0.0);
    auto_declare<double>("sigma_W", 0.0);
    auto_declare<std::vector<double>>("Kp_init", {-10.0, -15.0, -10.0, -5.0});
    auto_declare<std::vector<double>>("Kd_init", {-1.0, -5.0, -2.0, 0.0});
    auto_declare<std::vector<double>>("Kp_max", {60.0, 300.0, 150.0, 15.0});
    auto_declare<std::vector<double>>("Kd_max", {5.0, 50.0, 15.0, 0.3});
    auto_declare<std::vector<double>>("Kr_init", {10.0, 15.0, 10.0, 5.0});
    auto_declare<double>("Kr_max", 300.0);
    auto_declare<std::vector<double>>("W_max", {3.0, 50.0, 30.0, 0.5});
    auto_declare<std::vector<double>>("tau_limit", {150.0, 150.0, 100.0, 50.0});
    auto_declare<double>("goal_tolerance", 0.05);
    auto_declare<double>("goal_time_tolerance", 5.0);
    auto_declare<int>("diag_decimation", 10);
    auto_declare<std::string>("weights_file", "");
    auto_declare<bool>("load_weights_on_activate", false);
  } catch (const std::exception &ex) {
    fprintf(stderr, "MracController: exception while declaring parameters: %s\n", ex.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool MracController::readParameters()
{
  auto node = get_node();
  auto vec4 = [&](const std::string &name, Vec4 &out) {
    const std::vector<double> v = node->get_parameter(name).as_double_array();
    if (v.size() != static_cast<std::size_t>(N_JOINTS)) {
      RCLCPP_ERROR(node->get_logger(), "Parameter '%s' must have %d values, got %zu", name.c_str(), N_JOINTS,
                   v.size());
      return false;
    }
    for (int i = 0; i < N_JOINTS; i++) {
      if (!std::isfinite(v[i])) {
        RCLCPP_ERROR(node->get_logger(), "Parameter '%s' has a non-finite value", name.c_str());
        return false;
      }
      out(i) = v[i];
    }
    return true;
  };

  joints_ = node->get_parameter("joints").as_string_array();
  if (joints_.size() != static_cast<std::size_t>(N_JOINTS)) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' must list exactly %d joints", N_JOINTS);
    return false;
  }

  MracParams p;
  p.wn             = node->get_parameter("wn").as_double();
  p.zeta           = node->get_parameter("zeta").as_double();
  p.sign_b         = node->get_parameter("sign_b").as_double();
  p.decentralized  = node->get_parameter("decentralized").as_bool();
  p.robust_enabled = node->get_parameter("robust_enabled").as_bool();
  p.sigma          = node->get_parameter("sigma").as_double();
  p.sigma_W        = node->get_parameter("sigma_W").as_double();
  p.Kr_max         = node->get_parameter("Kr_max").as_double();
  if (!vec4("gamma_Kx", p.gamma_Kx) || !vec4("gamma_Kr", p.gamma_Kr) || !vec4("gamma_W", p.gamma_W) ||
      !vec4("Kp_init", p.Kp_init) || !vec4("Kd_init", p.Kd_init) || !vec4("Kr_init", p.Kr_init) ||
      !vec4("tau_limit", p.tau_limit) || !vec4("W_max", p.W_max) || !vec4("Kp_max", p.Kp_max) || !vec4("Kd_max", p.Kd_max)) {
    return false;
  }
  if (p.wn <= 0.0 || p.zeta <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "wn and zeta must be positive");
    return false;
  }
  if ((p.tau_limit.array() <= 0.0).any()) {
    RCLCPP_ERROR(node->get_logger(), "tau_limit values must be positive");
    return false;
  }
  params_ = p;

  goal_tolerance_           = node->get_parameter("goal_tolerance").as_double();
  goal_time_tolerance_      = node->get_parameter("goal_time_tolerance").as_double();
  diag_decimation_          = std::max<int>(1, static_cast<int>(node->get_parameter("diag_decimation").as_int()));
  weights_file_             = node->get_parameter("weights_file").as_string();
  load_weights_on_activate_ = node->get_parameter("load_weights_on_activate").as_bool();
  return true;
}

InterfaceConfiguration MracController::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto &j : joints_) cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  return cfg;
}

InterfaceConfiguration MracController::state_interface_configuration() const
{
  // Order matters: positions of all joints first, then velocities.
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto &j : joints_) cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  for (const auto &j : joints_) cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  return cfg;
}

CallbackReturn MracController::on_configure(const rclcpp_lifecycle::State &)
{
  if (!readParameters()) return CallbackReturn::ERROR;
  auto node = get_node();

  {
    std::lock_guard<std::mutex> lk(core_mx_);
    core_.configure(params_);
  }
  robust_enabled_.store(params_.robust_enabled);

  // Diagnostics
  tau_pub_      = node->create_publisher<Float64Array>("/mrac/tau", 10);
  khat_pub_     = node->create_publisher<Float64Array>("/mrac/K_hat", 10);
  w_pub_        = node->create_publisher<Float64Array>("/mrac/W_hat", 10);
  err_pub_      = node->create_publisher<Float64Array>("/mrac/error", 10);
  traj_err_pub_ = node->create_publisher<Float64Array>("/mrac/traj_error", 10);
  uad_pub_      = node->create_publisher<Float64Array>("/mrac/u_ad", 10);
  ref_pub_      = node->create_publisher<Float64Array>("/mrac/ref_model", 10);
  lyap_pub_     = node->create_publisher<Float64Array>("/mrac/lyapunov_V", 10);
  mode_pub_     = node->create_publisher<std_msgs::msg::Int32>("/mrac/mode", 10);

  tau_msg_.data.assign(N_JOINTS, 0.0);
  khat_msg_.data.assign(3 * N_JOINTS, 0.0);
  w_msg_.data.assign(N_BASIS * N_JOINTS, 0.0);
  err_msg_.data.assign(2 * N_JOINTS, 0.0);
  traj_err_msg_.data.assign(N_JOINTS, 0.0);
  uad_msg_.data.assign(N_JOINTS, 0.0);
  ref_msg_.data.assign(2 * N_JOINTS, 0.0);
  lyap_msg_.data.assign(N_JOINTS, 0.0);

  robust_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/mrac/robust_enable", 10, [this](std_msgs::msg::Bool::SharedPtr msg) { robust_enabled_.store(msg->data); });

  // Weights save / load
  save_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/save_weights",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        MracWeights w;
        {
          std::lock_guard<std::mutex> lk(core_mx_);
          w = core_.weights();
        }
        std::string err;
        const std::string path = weightsPath();
        res->success = saveWeights(path, w, err);
        res->message = res->success ? "saved weights to " + path : "save failed: " + err;
        RCLCPP_INFO(get_node()->get_logger(), "%s", res->message.c_str());
      });

  load_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/load_weights",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        MracWeights w;
        std::string err;
        const std::string path = weightsPath();
        res->success = loadWeights(path, w, err);
        if (res->success) {
          std::lock_guard<std::mutex> lk(core_mx_);
          core_.setWeights(w);
        }
        res->message = res->success ? "loaded weights from " + path : "load failed: " + err;
        RCLCPP_INFO(get_node()->get_logger(), "%s", res->message.c_str());
      });

  // Trajectory action
  action_server_ = rclcpp_action::create_server<FollowJT>(
      node, "~/follow_joint_trajectory",
      std::bind(&MracController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MracController::handleCancel, this, std::placeholders::_1),
      std::bind(&MracController::handleAccepted, this, std::placeholders::_1));

  goal_timer_ = node->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MracController::goalMonitor, this));

  RCLCPP_INFO(node->get_logger(),
              "MRAC configured: wn=%.2f zeta=%.2f, %s gains, robust term %s, %d basis terms", params_.wn,
              params_.zeta, params_.decentralized ? "per-joint" : "full-matrix",
              params_.robust_enabled ? "on" : "off", N_BASIS);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MracController::on_activate(const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != static_cast<std::size_t>(2 * N_JOINTS) ||
      command_interfaces_.size() != static_cast<std::size_t>(N_JOINTS)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %d state and %d command interfaces, got %zu and %zu",
                 2 * N_JOINTS, N_JOINTS, state_interfaces_.size(), command_interfaces_.size());
    return CallbackReturn::ERROR;
  }

  if (load_weights_on_activate_) {
    MracWeights w;
    std::string err;
    const std::string path = weightsPath();
    if (loadWeights(path, w, err)) {
      std::lock_guard<std::mutex> lk(core_mx_);
      core_.setWeights(w);
      RCLCPP_INFO(get_node()->get_logger(), "Loaded weights from %s", path.c_str());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Could not load weights from %s (%s); starting from initial values",
                  path.c_str(), err.c_str());
    }
  }

  Vec4 q = Vec4::Zero(), qd = Vec4::Zero();
  readState(q, qd);
  {
    std::lock_guard<std::mutex> lk(core_mx_);
    core_.resetReference(q, qd);
  }
  q_hold_ = q;
  q_goal_ = q;
  tau_last_.setZero();
  active_.reset();
  mode_ = HOLD;
  diag_count_ = 0;
  active_flag_.store(true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MracController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_flag_.store(false);
  writeEffort(Vec4::Zero());
  mode_ = IDLE;

  std::lock_guard<std::mutex> lk(goal_mx_);
  if (current_gh_) {
    auto res = std::make_shared<FollowJT::Result>();
    res->error_code = FollowJT::Result::INVALID_GOAL;
    res->error_string = "controller deactivated";
    current_gh_->abort(res);
    current_gh_.reset();
  }
  return CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════════════════
//  Control loop
// ════════════════════════════════════════════════════════════════════════════
bool MracController::readState(Vec4 &q, Vec4 &qd)
{
  bool ok = true;
  for (int i = 0; i < N_JOINTS; i++) {
    const auto p = state_interfaces_[i].get_optional();
    const auto v = state_interfaces_[N_JOINTS + i].get_optional();
    if (p && std::isfinite(*p)) q_last_(i) = *p; else ok = false;
    if (v && std::isfinite(*v)) qd_last_(i) = *v; else ok = false;
  }
  q = q_last_;
  qd = qd_last_;
  return ok;
}

void MracController::writeEffort(const Vec4 &tau)
{
  for (int i = 0; i < N_JOINTS && i < static_cast<int>(command_interfaces_.size()); i++)
    (void)command_interfaces_[i].set_value(tau(i));
}

controller_interface::return_type MracController::update(const rclcpp::Time &, const rclcpp::Duration &period)
{
  double dt = period.seconds();
  if (!(dt > 0.0) || dt > 0.1) {
    const unsigned int rate = get_update_rate();
    dt = rate > 0 ? 1.0 / static_cast<double>(rate) : 0.01;
  }

  Vec4 q, qd;
  readState(q, qd);

  // New command from the action side?
  const std::shared_ptr<Command> cmd = *rt_cmd_.readFromRT();
  const bool new_cmd = cmd && cmd->id != active_id_;

  std::unique_lock<std::mutex> lk(core_mx_, std::try_to_lock);
  if (!lk.owns_lock()) {          // a save/load is copying weights: repeat the last torque
    writeEffort(tau_last_);
    return controller_interface::return_type::OK;
  }

  if (new_cmd) {
    active_id_ = cmd->id;
    active_ = cmd;
    if (cmd->hold) {
      q_hold_ = q;
      mode_ = HOLD;
    } else {
      t_traj_ = 0.0;
      for (int i = 0; i < N_JOINTS; i++) q_goal_(i) = cmd->traj.points.back().positions[cmd->col[i]];
      core_.resetReference(q, qd);
      mode_ = EXECUTING;
    }
  }

  core_.setRobustEnabled(robust_enabled_.load());

  Vec4 q_d = q_hold_, r = q_hold_;
  if (mode_ == EXECUTING) {
    Vec4 v_d = Vec4::Zero(), a_d = Vec4::Zero();
    for (int i = 0; i < N_JOINTS; i++) {
      Sample s;
      sampleTraj(active_->traj, t_traj_, active_->col[i], s);   // validated when the goal was accepted
      q_d(i) = s.q;
      v_d(i) = s.dq;
      a_d(i) = s.ddq;
    }
    r = core_.ref().precompensate(q_d, v_d, a_d);
  }

  const MracOutput out = core_.update(q, qd, r, dt, mode_ == EXECUTING);
  tau_last_ = out.tau;
  writeEffort(out.tau);

  if (mode_ == EXECUTING) {
    t_traj_ += dt;
    if (t_traj_ >= active_->t_end) {
      const bool in_tol = (q - q_goal_).cwiseAbs().maxCoeff() <= goal_tolerance_;
      const bool timed_out = t_traj_ > active_->t_end + goal_time_tolerance_;
      if (in_tol || timed_out) {
        q_hold_ = q_goal_;
        mode_ = HOLD;
        done_code_.store(in_tol ? DONE_SUCCESS : DONE_TOLERANCE);
        done_id_.store(active_->id);
      }
    }
  }

  if (++diag_count_ >= diag_decimation_) {
    diag_count_ = 0;
    publishDiagnostics(out, q, q_d);
  }
  return controller_interface::return_type::OK;
}

void MracController::publishDiagnostics(const MracOutput &out, const Vec4 &q, const Vec4 &q_d)
{
  const MracWeights &w = core_.weights();
  const Eigen::Matrix2d &P = core_.ref().P;
  for (int i = 0; i < N_JOINTS; i++) {
    tau_msg_.data[i]          = out.tau(i);
    khat_msg_.data[3 * i]     = w.Kx(i, i);
    khat_msg_.data[3 * i + 1] = w.Kx(i, N_JOINTS + i);
    khat_msg_.data[3 * i + 2] = w.Kr(i, i);
    for (int b = 0; b < N_BASIS; b++) w_msg_.data[i * N_BASIS + b] = w.W(b, i);
    err_msg_.data[2 * i]      = out.e(i);
    err_msg_.data[2 * i + 1]  = out.ed(i);
    traj_err_msg_.data[i]     = q(i) - q_d(i);
    uad_msg_.data[i]          = out.u_ad(i);
    ref_msg_.data[2 * i]      = out.q_m(i);
    ref_msg_.data[2 * i + 1]  = out.qd_m(i);
    const Eigen::Vector2d ev(out.e(i), out.ed(i));
    lyap_msg_.data[i]         = ev.dot(P * ev);
  }
  tau_pub_->publish(tau_msg_);
  khat_pub_->publish(khat_msg_);
  w_pub_->publish(w_msg_);
  err_pub_->publish(err_msg_);
  traj_err_pub_->publish(traj_err_msg_);
  uad_pub_->publish(uad_msg_);
  ref_pub_->publish(ref_msg_);
  lyap_pub_->publish(lyap_msg_);
  std_msgs::msg::Int32 mode;
  mode.data = mode_;
  mode_pub_->publish(mode);
}

std::string MracController::weightsPath() const
{
  const char *home = std::getenv("HOME");
  const std::string h = home ? home : ".";
  if (weights_file_.empty()) return h + "/.ros/mrac_weights.yaml";
  if (weights_file_.rfind("~/", 0) == 0) return h + weights_file_.substr(1);
  return weights_file_;
}

// ════════════════════════════════════════════════════════════════════════════
//  Action server (runs in the controller manager's executor, not in update())
// ════════════════════════════════════════════════════════════════════════════
rclcpp_action::GoalResponse MracController::handleGoal(const rclcpp_action::GoalUUID &,
                                                       std::shared_ptr<const FollowJT::Goal> goal)
{
  auto logger = get_node()->get_logger();
  if (!active_flag_.load()) {
    RCLCPP_WARN(logger, "Goal rejected: controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto &traj = goal->trajectory;
  if (traj.points.empty()) {
    RCLCPP_WARN(logger, "Goal rejected: trajectory has no points");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto &j : joints_) {
    if (std::find(traj.joint_names.begin(), traj.joint_names.end(), j) == traj.joint_names.end()) {
      RCLCPP_WARN(logger, "Goal rejected: trajectory does not contain joint '%s'", j.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  double t_prev = -1.0;
  for (const auto &p : traj.points) {
    if (p.positions.size() != traj.joint_names.size() ||
        (!p.velocities.empty() && p.velocities.size() != traj.joint_names.size())) {
      RCLCPP_WARN(logger, "Goal rejected: a trajectory point has the wrong number of values");
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (double v : p.positions) {
      if (!std::isfinite(v)) {
        RCLCPP_WARN(logger, "Goal rejected: non-finite position");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    for (double v : p.velocities) {
      if (!std::isfinite(v)) {
        RCLCPP_WARN(logger, "Goal rejected: non-finite velocity");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    const double t = pointTime(p);
    if (t < t_prev) {
      RCLCPP_WARN(logger, "Goal rejected: time_from_start is not increasing");
      return rclcpp_action::GoalResponse::REJECT;
    }
    t_prev = t;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MracController::handleCancel(std::shared_ptr<GoalHandleFJT>)
{
  // The goal monitor sees is_canceling() and finishes the goal.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MracController::handleAccepted(std::shared_ptr<GoalHandleFJT> gh)
{
  auto cmd = std::make_shared<Command>();
  cmd->traj = gh->get_goal()->trajectory;
  for (int i = 0; i < N_JOINTS; i++) {
    const auto it = std::find(cmd->traj.joint_names.begin(), cmd->traj.joint_names.end(), joints_[i]);
    cmd->col[i] = static_cast<std::size_t>(std::distance(cmd->traj.joint_names.begin(), it));
  }
  cmd->t_end = trajectoryEndTime(cmd->traj);

  std::lock_guard<std::mutex> lk(goal_mx_);
  if (current_gh_) {              // preempt the goal that is still running
    auto res = std::make_shared<FollowJT::Result>();
    res->error_code = FollowJT::Result::INVALID_GOAL;
    res->error_string = "preempted by a new goal";
    current_gh_->abort(res);
  }
  cmd->id = ++next_id_;
  current_gh_ = gh;
  current_id_ = cmd->id;
  rt_cmd_.writeFromNonRT(cmd);
}

void MracController::sendHoldCommand()
{
  auto cmd = std::make_shared<Command>();
  cmd->hold = true;
  cmd->id = ++next_id_;
  rt_cmd_.writeFromNonRT(cmd);
}

// 50 Hz: finishes the active goal once the control loop reports it done, or
// when the client asked to cancel it.
void MracController::goalMonitor()
{
  std::lock_guard<std::mutex> lk(goal_mx_);
  if (!current_gh_) return;

  auto res = std::make_shared<FollowJT::Result>();
  if (current_gh_->is_canceling()) {
    sendHoldCommand();
    res->error_code = FollowJT::Result::SUCCESSFUL;
    res->error_string = "canceled; holding current position";
    current_gh_->canceled(res);
    current_gh_.reset();
    return;
  }

  if (done_id_.load() != current_id_) return;
  if (done_code_.load() == DONE_SUCCESS) {
    res->error_code = FollowJT::Result::SUCCESSFUL;
    current_gh_->succeed(res);
  } else {
    res->error_code = FollowJT::Result::GOAL_TOLERANCE_VIOLATED;
    res->error_string = "goal tolerance not reached within goal_time_tolerance";
    RCLCPP_WARN(get_node()->get_logger(), "%s", res->error_string.c_str());
    current_gh_->abort(res);
  }
  current_gh_.reset();
}

}  // namespace mrac_controller

PLUGINLIB_EXPORT_CLASS(mrac_controller::MracController, controller_interface::ControllerInterface)
