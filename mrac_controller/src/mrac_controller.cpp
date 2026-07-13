/*
 * mrac_controller.cpp
 * ════════════════════════════════════════════════════════════════════════════
 * Direct Robust MRAC for 4-DOF Robotic Arm
 * Reference: Dr. Bharat Verma §17.3 (Direct MRAC) + §19.2 (Robust/Uncertainty)
 *
 * ── CONTROLLER MODES ─────────────────────────────────────────────────────────
 *
 *  MODE 0 — IDLE  (executing_=false, holding_=false)
 *    Before the first joint state arrives. No torque is published.
 *
 *  MODE 1 — EXECUTING  (executing_=true, holding_=false)
 *    Active while a FollowJointTrajectory goal is running.
 *    r_goal  = final waypoint of the trajectory (constant step target).
 *    r_vec   = Hermite-interpolated position at the current time t, used as
 *              the instantaneous reference fed to the reference model and the
 *              K̂_r adaptation law so that both track the smooth path, not
 *              just the endpoint.
 *    All three adaptation laws run: K̂_x, K̂_r, Ŵ.
 *
 *  MODE 2 — MRAC HOLD  (executing_=false, holding_=true)
 *    Active between trajectory goals and during Python dwell time.
 *    r = q_hold_ = the last commanded goal pose, held constant every tick.
 *    K̂_x and Ŵ continue adapting — the arm learns gravity compensation at
 *    this configuration during the dwell, so the next trajectory starts with
 *    near-zero initial error.
 *    K̂_r is FROZEN: r is constant so the product e·r carries no new
 *    directional information, and updating K̂_r would corrupt the gravity
 *    compensation being learned by K̂_x.
 *    By Barbalat's Lemma (r constant, e→0), K̂_x converges.
 *
 * ── MODE TRANSITION — NO-TORQUE-GAP GUARANTEE ────────────────────────────────
 *    When a new goal arrives the accepted callback:
 *      1. Loads the trajectory under traj_mx_ (no flag changes yet).
 *      2. Resets the reference model to the current actual position.
 *      3. Sets executing_ = true   ← control loop immediately uses new traj.
 *      4. Sets holding_  = false   ← hold ends AFTER executing is armed.
 *    This ordering ensures the control loop never observes both flags false
 *    simultaneously, which would drop torque for one or more ticks and let
 *    gravity pull the arm down between poses.
 *
 * ── REFERENCE MODEL (§16.3.2) ────────────────────────────────────────────────
 *   G_ref(s) = wn² / (s² + 2ζwn·s + wn²)   — shared second-order model,
 *   one independent state column per joint.
 *   Am = [[0, 1], [−wn², −2ζwn]]
 *   Bm = [0, wn²]ᵀ
 *   P  = analytical solution of  P·Am + Amᵀ·P = −I  (used in Lyapunov proof)
 *   Integration: Forward Euler (matches previous functional version).
 *
 * ── CONTROL LAW ──────────────────────────────────────────────────────────────
 *   e      = q − q_m                                    (4×1)
 *   u_mrac = K̂_x ⊙ q + K̂_r ⊙ r                      (§17.3, element-wise)
 *   u_ad   = −Ŵᵀ · α(q, q̇)  per joint                 (§19.2, robust term)
 *   τ      = clamp(u_mrac + u_ad,  ±TAU_LIM[i])
 *
 * ── ADAPTATION LAWS ──────────────────────────────────────────────────────────
 *   K̂̇_x[i] = −Γ_Kx · sign(b) · e[i] · q[i] − σ · K̂_x[i]
 *             [sigma modification/leakage; always active, both modes]
 *
 *   K̂̇_r[i] = −Γ_Kr · sign(b) · e[i] · r[i] − σ · K̂_r[i]
 *             [EXECUTING only]
 *
 *   Ŵ̇[:,i]  =  Γ_W · (eᵀ·P·B_ss)[i] · α(q,q̇) − σ_W · Ŵ[:,i]
 *             [robust_enabled=true]
 *             B_ss is hardcoded [0, 1] to match previous functional version.
 *
 * ── BASIS FUNCTION α(q, q̇)  — dim=22  (§19.1.1) ────────────────────────────
 *   [1,  q0..q3,  sin(q0)..sin(q3),  cos(q0)..cos(q3),
 *    q0·q1, q1·q2, q2·q3,
 *    q̇0..q̇3,  tanh(q̇0)..tanh(q̇1)]
 *   tanh(q̇) for first two joints only to match previous version and monitor.
 *   Total: 1 + 4 + 4 + 4 + 3 + 4 + 2 = 22. N_BASIS must equal 22.
 *
 * ── ROS2 INTERFACES ─────────────────────────────────────────────────────────
 *   Action server  /mrac_arm_controller/follow_joint_trajectory
 *   Sub            /joint_states
 *   Pub            /arm_effort_controller/commands   (effort commands, 4 joints)
 *   Pub            /mrac/K_hat        [Kx0,Kr0, Kx1,Kr1, Kx2,Kr2, Kx3,Kr3]
 *   Pub            /mrac/W_hat        [Ŵ[:,0](22), Ŵ[:,1](22), ...]  dim=88
 *   Pub            /mrac/error        [e0,ė0, e1,ė1, e2,ė2, e3,ė3]
 *   Pub            /mrac/traj_error   [q[i]−r_goal[i]]  meaningful in exec only
 *   Pub            /mrac/u_ad         [u_ad per joint]
 *   Pub            /mrac/ref_model    [q_m0,q̇_m0, ...]
 *   Pub            /mrac/lyapunov_V   [V_i = eᵀPe + ‖Ŵ_i‖²/Γ_W]
 *   Pub            /mrac/mode         Int32: 0=idle, 1=executing, 2=mrac_hold
 *   Sub            /mrac/robust_enable  std_msgs/Bool — runtime toggle
 * ════════════════════════════════════════════════════════════════════════════
 */

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>

using FollowJT        = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT   = rclcpp_action::ServerGoalHandle<FollowJT>;

// ════════════════════════════════════════════════════════════════════════════
//  COMPILE-TIME CONSTANTS
// ════════════════════════════════════════════════════════════════════════════
static constexpr int    N_JOINTS = 4;
// N_BASIS = 1 + 4 + 4 + 4 + 3 + 4 + 2 = 22
// Matches previous version and mrac_monitor.py
static constexpr int    N_BASIS  = 22;
static constexpr double DT       = 0.001;   // 1 kHz control loop

// Per-joint torque limits [Nm]. Joint 4 (wrist) is physically weaker.
static constexpr double TAU_LIM[N_JOINTS] = {250.0, 250.0, 250.0, 80.0};

static const std::string JOINT_NAMES[N_JOINTS] = {
    "joint1", "joint2", "joint3", "joint4"};

using Vec4  = Eigen::Matrix<double, N_JOINTS, 1>;
using VecB  = Eigen::Matrix<double, N_BASIS,  1>;
using MatWB = Eigen::Matrix<double, N_BASIS,  N_JOINTS>;  // W_hat layout

// ════════════════════════════════════════════════════════════════════════════
//  BASIS FUNCTION α(q, q̇)  — 22 features
// ════════════════════════════════════════════════════════════════════════════
static VecB computeAlpha(const Vec4 &q, const Vec4 &qd)
{
  VecB a;
  a <<  1.0,                                              // bias
        q(0),  q(1),  q(2),  q(3),                       // linear position
        std::sin(q(0)), std::sin(q(1)),                   // gravity shape
        std::sin(q(2)), std::sin(q(3)),
        std::cos(q(0)), std::cos(q(1)),
        std::cos(q(2)), std::cos(q(3)),
        q(0)*q(1), q(1)*q(2), q(2)*q(3),                 // joint coupling
        qd(0), qd(1), qd(2), qd(3),                      // viscous friction
        std::tanh(qd(0)), std::tanh(qd(1));               // Coulomb friction (first 2 joints)
  return a;
}

// ════════════════════════════════════════════════════════════════════════════
//  REFERENCE MODEL  (§16.3.2)
// ════════════════════════════════════════════════════════════════════════════
struct ReferenceModel
{
  double wn = 1.0, zeta = 1.0;
  Eigen::Matrix2d  Am;
  Eigen::Vector2d  Bm;
  Eigen::Matrix2d  P;
  Eigen::Matrix<double, 2, N_JOINTS> X_m;

  ReferenceModel() = default;

  ReferenceModel(double wn_, double zeta_) : wn(wn_), zeta(zeta_)
  {
    Am << 0.0,      1.0,
          -wn*wn,  -2.0*zeta*wn;
    Bm << 0.0,
          wn*wn;

    // Analytical solution of  P·Am + Amᵀ·P = −I
    double p12 = 1.0 / (2.0 * wn * wn);
    double p22 = (1.0 + 2.0 * p12) / (4.0 * zeta * wn);
    double p11 = 2.0 * zeta * wn * p12 + wn * wn * p22;
    P << p11, p12,
         p12, p22;

    X_m.setZero();
  }

  void reset(const Vec4 &q0, const Vec4 &qd0)
  {
    for (int i = 0; i < N_JOINTS; i++) {
      X_m(0, i) = q0(i);
      X_m(1, i) = qd0(i);
    }
  }

  // Forward Euler step (matches previous functional version)
  void step(const Vec4 &r)
  {
    Eigen::Matrix<double, 2, N_JOINTS> dX;
    for (int i = 0; i < N_JOINTS; i++)
      dX.col(i) = Am * X_m.col(i) + Bm * r(i);
    X_m += dX * DT;
  }

  Vec4 q_m()  const { return X_m.row(0).transpose(); }
  Vec4 qd_m() const { return X_m.row(1).transpose(); }
};

// ════════════════════════════════════════════════════════════════════════════
//  TRAJECTORY SAMPLER — cubic Hermite interpolation
// ════════════════════════════════════════════════════════════════════════════
struct Sample { double q = 0.0, dq = 0.0; };

static bool sampleTraj(const trajectory_msgs::msg::JointTrajectory &traj,
                        double t, int ji, Sample &s)
{
  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &pts = traj.points;
  if (pts.empty()) return false;

  double t_end = rclcpp::Duration(pts.back().time_from_start).seconds();
  if (t > t_end + 0.5) return false;

  if (t >= t_end) {
    s.q  = pts.back().positions.empty() ? 0.0 : pts.back().positions[ji];
    s.dq = 0.0;
    return true;
  }

  int k = static_cast<int>(pts.size()) - 2;
  for (int i = 0; i < static_cast<int>(pts.size()) - 1; i++) {
    if (t < rclcpp::Duration(pts[i+1].time_from_start).seconds()) {
      k = i; break;
    }
  }

  double t0  = rclcpp::Duration(pts[k].time_from_start).seconds();
  double t1  = rclcpp::Duration(pts[k+1].time_from_start).seconds();
  double seg = std::max(t1 - t0, 1e-6);
  double tau = std::clamp((t - t0) / seg, 0.0, 1.0);

  double h00 =  2*tau*tau*tau - 3*tau*tau + 1;
  double h10 =    tau*tau*tau - 2*tau*tau + tau;
  double h01 = -2*tau*tau*tau + 3*tau*tau;
  double h11 =    tau*tau*tau -   tau*tau;

  double v0 = pts[k].velocities.empty()   ? 0.0 : pts[k].velocities[ji];
  double v1 = pts[k+1].velocities.empty() ? 0.0 : pts[k+1].velocities[ji];

  s.q = h00 * pts[k].positions[ji]   + h10 * seg * v0
      + h01 * pts[k+1].positions[ji] + h11 * seg * v1;

  double dh00 = ( 6*tau*tau - 6*tau       ) / seg;
  double dh10 = ( 3*tau*tau - 4*tau + 1.0 ) / seg;
  double dh01 = (-6*tau*tau + 6*tau       ) / seg;
  double dh11 = ( 3*tau*tau - 2*tau       ) / seg;

  s.dq = dh00 * pts[k].positions[ji]   + dh10 * seg * v0
       + dh01 * pts[k+1].positions[ji] + dh11 * seg * v1;

  return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  MRAC CONTROLLER NODE
// ════════════════════════════════════════════════════════════════════════════
class MRACController : public rclcpp::Node
{
public:
  MRACController() : Node("robust_mrac_controller")
  {
    declare_parameter("wn",           5.0);
    declare_parameter("zeta",         0.85);
    declare_parameter("sign_b",       1.0);
    declare_parameter("gamma_Kx",     std::vector<double>{0.3, 0.3, 0.3, 0.3});
    declare_parameter("gamma_Kr",     std::vector<double>{0.3, 0.3, 0.3, 0.3});
    declare_parameter("Khat_x_init",  std::vector<double>{-10.0, -10.0, -10.0, -5.0});
    declare_parameter("Khat_r_init",  std::vector<double>{ 10.0,  10.0,  10.0,  5.0});
    declare_parameter("Kx_max",       300.0);
    declare_parameter("Kr_max",       300.0);
    declare_parameter("robust_enabled", true);
    declare_parameter("gamma_W",      std::vector<double>{0.5, 0.5, 0.5, 0.5});
    declare_parameter("W_max",        50.0);
    declare_parameter("sigma",        0.0);
    declare_parameter("sigma_W",      0.0);

    const double wn   = get_parameter("wn").as_double();
    const double zeta = get_parameter("zeta").as_double();
    sign_b_         = get_parameter("sign_b").as_double();
    const std::vector<double> gKx  = get_parameter("gamma_Kx").as_double_array();
    const std::vector<double> gKr  = get_parameter("gamma_Kr").as_double_array();
    const std::vector<double> Kx_i = get_parameter("Khat_x_init").as_double_array();
    const std::vector<double> Kr_i = get_parameter("Khat_r_init").as_double_array();
    Kx_max_         = get_parameter("Kx_max").as_double();
    Kr_max_         = get_parameter("Kr_max").as_double();
    robust_enabled_ = get_parameter("robust_enabled").as_bool();
    const std::vector<double> gW   = get_parameter("gamma_W").as_double_array();
    W_max_          = get_parameter("W_max").as_double();
    sigma_          = get_parameter("sigma").as_double();
    sigma_W_        = get_parameter("sigma_W").as_double();

    ref_ = ReferenceModel(wn, zeta);

    for (int i = 0; i < N_JOINTS; i++) {
      gamma_Kx_(i) = (i < (int)gKx.size()) ? gKx[i] : gKx.back();
      gamma_Kr_(i) = (i < (int)gKr.size()) ? gKr[i] : gKr.back();
      gamma_W_(i)  = (i < (int)gW.size())  ? gW[i]  : gW.back();
      K_hat_x_(i) = Kx_i[i];
      K_hat_r_(i) = Kr_i[i];
    }
    W_hat_.setZero();
    q_.setZero();
    qd_.setZero();
    q_hold_.setZero();

    // ── Publishers ────────────────────────────────────────────────────
    tau_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_effort_controller/commands", 10);
    Khat_pub_     = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/K_hat",       10);
    W_pub_        = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/W_hat",       10);
    err_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/error",       10);
    traj_err_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/traj_error",  10);
    uad_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/u_ad",        10);
    ref_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/ref_model",   10);
    lyap_pub_     = create_publisher<std_msgs::msg::Float64MultiArray>("/mrac/lyapunov_V",  10);
    mode_pub_     = create_publisher<std_msgs::msg::Int32>("/mrac/mode", 10);

    robust_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mrac/robust_enable", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) {
          robust_enabled_ = msg->data;
        });

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(state_mx_);
          if (idx_map_.empty()) {
            for (int i = 0; i < N_JOINTS; i++)
              for (size_t k = 0; k < msg->name.size(); k++)
                if (msg->name[k] == JOINT_NAMES[i])
                  idx_map_[JOINT_NAMES[i]] = static_cast<int>(k);
          }
          for (int i = 0; i < N_JOINTS; i++) {
            std::map<std::string, int>::iterator it = idx_map_.find(JOINT_NAMES[i]);
            if (it == idx_map_.end()) continue;
            int k = it->second;
            if (k < static_cast<int>(msg->position.size())) q_(i)  = msg->position[k];
            if (k < static_cast<int>(msg->velocity.size())) qd_(i) = msg->velocity[k];
          }
          if (!ready_) {
            ref_.reset(q_, qd_);
            q_hold_ = q_;
            ready_  = true;
            holding_ = true;
          }
        });

    action_server_ = rclcpp_action::create_server<FollowJT>(
        this, "/mrac_arm_controller/follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJT::Goal> goal) -> rclcpp_action::GoalResponse {
          if (goal->trajectory.points.empty()) return rclcpp_action::GoalResponse::REJECT;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandleFJT>) -> rclcpp_action::CancelResponse {
          executing_ = false;
          { std::lock_guard<std::mutex> lk(state_mx_); q_hold_ = q_; }
          holding_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandleFJT> gh) {
          {
            std::lock_guard<std::mutex> lk(traj_mx_);
            goal_handle_  = gh;
            traj_         = gh->get_goal()->trajectory;
            traj_start_   = std::chrono::steady_clock::now();
            traj_col_.fill(-1);
            for (int i = 0; i < N_JOINTS; i++)
              for (size_t j = 0; j < traj_.joint_names.size(); j++)
                if (traj_.joint_names[j] == JOINT_NAMES[i]) {
                  traj_col_[i] = static_cast<int>(j); break;
                }
          }
          { std::lock_guard<std::mutex> lk(state_mx_); ref_.reset(q_, qd_); }
          {
            std::lock_guard<std::mutex> lk(mode_mx_);
            executing_ = true;
            holding_   = false;
          }
        });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&MRACController::controlLoop, this));
  }

private:
  void controlLoop()
  {
    if (!ready_) return;

    bool holding, executing;
    {
      std::lock_guard<std::mutex> lk(mode_mx_);
      holding   = holding_;
      executing = executing_;
    }

    if (holding && !executing) {
      runMRAC(q_hold_, /*adapt_Kr=*/false);
      publishMode(2);
      return;
    }

    if (!executing) {
      publishMode(0);
      return;
    }

    double t;
    trajectory_msgs::msg::JointTrajectory traj;
    std::array<int, N_JOINTS> traj_col;
    {
      std::lock_guard<std::mutex> lk(traj_mx_);
      t        = std::chrono::duration<double>(std::chrono::steady_clock::now() - traj_start_).count();
      traj     = traj_;
      traj_col = traj_col_;
    }

    double t_end = traj.points.empty() ? std::numeric_limits<double>::max() : rclcpp::Duration(traj.points.back().time_from_start).seconds();

    Vec4 r_goal, r_vec;
    {
      Vec4 q_snap;
      { std::lock_guard<std::mutex> lk(state_mx_); q_snap = q_; }
      for (int i = 0; i < N_JOINTS; i++) {
        int col = traj_col[i];
        if (col < 0) { r_goal(i) = q_snap(i); r_vec(i) = q_snap(i); continue; }
        r_goal(i) = traj.points.back().positions[col];
        Sample s{};
        sampleTraj(traj, t, col, s);
        r_vec(i) = s.q;
      }
    }

    runMRAC(r_vec, /*adapt_Kr=*/true);
    publishMode(1);

    if (t > t_end + 0.5) {
      q_hold_ = r_goal;
      {
        std::lock_guard<std::mutex> lk(mode_mx_);
        executing_ = false;
        holding_   = true;
      }
      std::shared_ptr<GoalHandleFJT> gh;
      { std::lock_guard<std::mutex> lk(traj_mx_); gh = goal_handle_; goal_handle_.reset(); }
      if (gh) {
        std::shared_ptr<FollowJT::Result> res = std::make_shared<FollowJT::Result>();
        res->error_code = FollowJT::Result::SUCCESSFUL;
        gh->succeed(res);
      }
    }
  }

  void runMRAC(const Vec4 &r, bool adapt_Kr)
  {
    Vec4 q, qd;
    { std::lock_guard<std::mutex> lk(state_mx_); q = q_; qd = qd_; }

    ref_.step(r);
    Vec4 q_m  = ref_.q_m();
    Vec4 qd_m = ref_.qd_m();
    Vec4 e  = q  - q_m;
    Vec4 ed = qd - qd_m;

    Vec4 u_mrac = K_hat_x_.array() * q.array() + K_hat_r_.array() * r.array();
    VecB alpha = computeAlpha(q, qd);

    Vec4 u_ad = Vec4::Zero();
    if (robust_enabled_)
      for (int i = 0; i < N_JOINTS; i++)
        u_ad(i) = -W_hat_.col(i).dot(alpha);

    Vec4 tau = u_mrac + u_ad;
    for (int i = 0; i < N_JOINTS; i++)
      tau(i) = std::clamp(tau(i), -TAU_LIM[i], TAU_LIM[i]);

    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(tau.data(), tau.data() + N_JOINTS);
      tau_pub_->publish(msg);
    }

    Vec4 dKx = (-gamma_Kx_.array() * sign_b_ * e.array() * q.array() - sigma_ * K_hat_x_.array()).matrix();
    K_hat_x_ += dKx * DT;
    for (int i = 0; i < N_JOINTS; i++) K_hat_x_(i) = std::clamp(K_hat_x_(i), -Kx_max_, Kx_max_);

    if (adapt_Kr) {
      Vec4 dKr = (-gamma_Kr_.array() * sign_b_ * e.array() * r.array() - sigma_ * K_hat_r_.array()).matrix();
      K_hat_r_ += dKr * DT;
      for (int i = 0; i < N_JOINTS; i++) K_hat_r_(i) = std::clamp(K_hat_r_(i), -Kr_max_, Kr_max_);
    }

    if (robust_enabled_) {
      // unit B vector [0, 1] 
      const Eigen::Vector2d B_ss(0.0, 1.0);
      for (int i = 0; i < N_JOINTS; i++) {
        Eigen::Vector2d ev(e(i), ed(i));
        double eTpB = ev.dot(ref_.P * B_ss);
        // Adaptation with smooth leakage (sigma modification)
        W_hat_.col(i) += (gamma_W_(i) * eTpB * DT) * alpha - (sigma_W_ * DT) * W_hat_.col(i);
        double wn_i = W_hat_.col(i).norm();
        if (wn_i > W_max_) W_hat_.col(i) *= (W_max_ / wn_i);
      }
    }

    publishDiagnostics(e, ed, u_ad, q_m, qd_m, q, r);
  }

  void publishMode(int mode)
  {
    std_msgs::msg::Int32 msg;
    msg.data = mode;
    mode_pub_->publish(msg);
  }

  void publishDiagnostics(const Vec4 &e, const Vec4 &ed, const Vec4 &u_ad,
                           const Vec4 &q_m, const Vec4 &qd_m, const Vec4 &q, const Vec4 &r)
  {
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(N_JOINTS * 2);
      for (int i = 0; i < N_JOINTS; i++) {
        msg.data[2*i]   = K_hat_x_(i);
        msg.data[2*i+1] = K_hat_r_(i);
      }
      Khat_pub_->publish(msg);
    }
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(N_BASIS * N_JOINTS);
      for (int i = 0; i < N_JOINTS; i++)
        for (int b = 0; b < N_BASIS; b++)
          msg.data[i * N_BASIS + b] = W_hat_(b, i);
      W_pub_->publish(msg);
    }
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(N_JOINTS * 2);
      for (int i = 0; i < N_JOINTS; i++) {
        msg.data[2*i]   = e(i);
        msg.data[2*i+1] = ed(i);
      }
      err_pub_->publish(msg);
    }
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(N_JOINTS);
      for (int i = 0; i < N_JOINTS; i++) msg.data[i] = q(i) - r(i);
      traj_err_pub_->publish(msg);
    }
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(u_ad.data(), u_ad.data() + N_JOINTS);
      uad_pub_->publish(msg);
    }
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(N_JOINTS * 2);
      for (int i = 0; i < N_JOINTS; i++) {
        msg.data[2*i]   = q_m(i);
        msg.data[2*i+1] = qd_m(i);
      }
      ref_pub_->publish(msg);
    }
    {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.resize(N_JOINTS);
      for (int i = 0; i < N_JOINTS; i++) {
        Eigen::Vector2d ev(e(i), ed(i));
        double V_e = ev.dot(ref_.P * ev);
        // V converges to a residual set (not zero) when sigma > 0 — expected behavior
        double V_W = robust_enabled_ ? W_hat_.col(i).squaredNorm() / (gamma_W_(i) + 1e-9) : 0.0;
        msg.data[i] = V_e + V_W;
      }
      lyap_pub_->publish(msg);
    }
  }

  ReferenceModel ref_;
  Vec4   K_hat_x_, K_hat_r_;
  MatWB  W_hat_;
  double sign_b_, Kx_max_, Kr_max_, sigma_, sigma_W_;
  Vec4   gamma_Kx_, gamma_Kr_, gamma_W_;
  bool   robust_enabled_;
  double W_max_;
  Vec4 q_, qd_, q_hold_;
  std::mutex state_mx_, traj_mx_, mode_mx_;
  std::map<std::string, int> idx_map_;
  bool ready_ = false, executing_ = false, holding_ = false;
  trajectory_msgs::msg::JointTrajectory    traj_;
  std::chrono::steady_clock::time_point    traj_start_;
  std::array<int, N_JOINTS>               traj_col_{};
  std::shared_ptr<GoalHandleFJT>           goal_handle_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      tau_pub_, Khat_pub_, W_pub_, err_pub_, traj_err_pub_, uad_pub_, ref_pub_, lyap_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           robust_sub_;
  rclcpp_action::Server<FollowJT>::SharedPtr                     action_server_;
  rclcpp::TimerBase::SharedPtr                                   timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MRACController>());
  rclcpp::shutdown();
  return 0;
}
