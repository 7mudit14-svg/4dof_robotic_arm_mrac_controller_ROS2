/*
 * mrac_core.hpp — ROS-free maths of the robust direct MRAC (multi-state form).
 *
 * State        x = [q; q̇]  (8),  reference r (4),  reference-model state x_m.
 * Error        e = x − x_m
 * Control law  τ = K̂x·x + K̂r·r − Ŵᵀ·α(q, q̇)          (notes §17.3 + §19.3)
 * Adaptation   s    = sign(b) · Bᵀ·P·e                  (per joint: p12·e + p22·ė)
 *              K̂x' = −Γx · s · xᵀ − σ (K̂x − K̂x0)
 *              K̂r' = −Γr · s · rᵀ − σ (K̂r − K̂r0)     (only when adapt_Kr)
 *              Ŵ'  =  ΓW · α · sᵀ − σW (Ŵ − Ŵ0)
 *
 * K̂x is 4×8 (columns 0..3 multiply q, columns 4..7 multiply q̇), K̂r is 4×4.
 * With decentralized = true only each joint's own entries are active
 * (Kp_i, Kd_i, Kr_i); with false the full matrices adapt.
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace mrac_controller
{

constexpr int N_JOINTS = 4;
constexpr int N_STATE  = 2 * N_JOINTS;
// α = [1, sin(q2), sin(q2+q3), tanh(q̇1)..tanh(q̇4)]
constexpr int N_BASIS  = 7;

using Vec4  = Eigen::Matrix<double, N_JOINTS, 1>;
using Vec8  = Eigen::Matrix<double, N_STATE, 1>;
using VecB  = Eigen::Matrix<double, N_BASIS, 1>;
using MatKx = Eigen::Matrix<double, N_JOINTS, N_STATE>;
using MatKr = Eigen::Matrix<double, N_JOINTS, N_JOINTS>;
using MatW  = Eigen::Matrix<double, N_BASIS, N_JOINTS>;   // column i = weights of joint i

// ── Basis α(q, q̇) ───────────────────────────────────────────────────────────
// Joints are indexed 0..3 in code (joint1..joint4 in the URDF).
// Joints 2 and 3 rotate about parallel axes, so the forearm's angle from the
// vertical is q2+q3 and gravity appears as sin(q2) and sin(q2+q3).
// Viscous friction (∝ q̇) is deliberately NOT a basis term: K̂x already has a
// gain on q̇, and two adaptive terms on the same signal are redundant.
inline VecB computeAlpha(const Vec4 &q, const Vec4 &qd)
{
  VecB a;
  a << 1.0,                                                   // constant offset
       std::sin(q(1)), std::sin(q(1) + q(2)),                 // gravity
       std::tanh(qd(0)), std::tanh(qd(1)),                    // Coulomb friction
       std::tanh(qd(2)), std::tanh(qd(3));
  return a;
}

// mask(b, i) = 1 if basis term b physically acts on joint i.
inline MatW basisMask()
{
  MatW m = MatW::Zero();
  m(1, 1) = 1.0;                 // sin(q2)     → joint2
  m(2, 1) = 1.0;                 // sin(q2+q3)  → joint2
  m(2, 2) = 1.0;                 // sin(q2+q3)  → joint3
  for (int i = 0; i < N_JOINTS; i++) {
    m(0, i)     = 1.0;           // 1           → every joint (removes steady offset)
    m(3 + i, i) = 1.0;           // tanh(q̇_i)  → joint i
  }
  return m;
}

// ── Reference model: wn² / (s² + 2ζwn·s + wn²), one copy per joint ──────────
struct ReferenceModel
{
  double wn = 1.0, zeta = 1.0;
  Eigen::Matrix2d Am = Eigen::Matrix2d::Zero();
  Eigen::Vector2d Bm = Eigen::Vector2d::Zero();
  Eigen::Matrix2d P  = Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 2, N_JOINTS> X_m = Eigen::Matrix<double, 2, N_JOINTS>::Zero();

  ReferenceModel() = default;

  ReferenceModel(double wn_, double zeta_) : wn(wn_), zeta(zeta_)
  {
    Am << 0.0, 1.0,
          -wn * wn, -2.0 * zeta * wn;
    Bm << 0.0, wn * wn;

    // Analytical solution of P·Am + Amᵀ·P = −I
    const double p12 = 1.0 / (2.0 * wn * wn);
    const double p22 = (1.0 + 2.0 * p12) / (4.0 * zeta * wn);
    const double p11 = 2.0 * zeta * wn * p12 + wn * wn * p22;
    P << p11, p12,
         p12, p22;
  }

  void reset(const Vec4 &q0, const Vec4 &qd0)
  {
    X_m.row(0) = q0.transpose();
    X_m.row(1) = qd0.transpose();
  }

  // RK4 step with r held constant over dt (stable at 100 Hz as well as 1 kHz).
  void step(const Vec4 &r, double dt)
  {
    for (int i = 0; i < N_JOINTS; i++) {
      const Eigen::Vector2d x  = X_m.col(i);
      const Eigen::Vector2d u  = Bm * r(i);
      const Eigen::Vector2d k1 = Am * x + u;
      const Eigen::Vector2d k2 = Am * (x + 0.5 * dt * k1) + u;
      const Eigen::Vector2d k3 = Am * (x + 0.5 * dt * k2) + u;
      const Eigen::Vector2d k4 = Am * (x + dt * k3) + u;
      X_m.col(i) = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
  }

  Vec4 q_m()  const { return X_m.row(0).transpose(); }
  Vec4 qd_m() const { return X_m.row(1).transpose(); }

  // Reference that makes q_m follow q_d without lag (model equation inverted).
  Vec4 precompensate(const Vec4 &q_d, const Vec4 &qd_d, const Vec4 &qdd_d) const
  {
    return q_d + (2.0 * zeta / wn) * qd_d + (1.0 / (wn * wn)) * qdd_d;
  }
};

// ── Parameters ──────────────────────────────────────────────────────────────
struct MracParams
{
  double wn = 3.0, zeta = 0.76;
  double sign_b = 1.0;
  bool   decentralized = true;
  bool   robust_enabled = true;

  Vec4 gamma_Kx = Vec4::Constant(50.0);
  Vec4 gamma_Kr = Vec4::Constant(50.0);
  Vec4 gamma_W  = Vec4::Constant(50.0);
  double sigma = 0.0, sigma_W = 0.0;

  Vec4 Kp_init = (Vec4() << -10.0, -15.0, -10.0, -5.0).finished();
  Vec4 Kd_init = (Vec4() <<  -1.0,  -5.0,  -2.0,  0.0).finished();
  Vec4 Kr_init = (Vec4() <<  10.0,  15.0,  10.0,  5.0).finished();

  double Kr_max = 300.0;
  // Bound on ‖Ŵ[:,i]‖ per joint, sized to the torques each joint's
  // uncertainty can need (gravity on joints 2 and 3, friction only on 1 and 4).
  Vec4 W_max = (Vec4() << 3.0, 50.0, 30.0, 0.5).finished();
  // Projection bounds on each joint's position and velocity gains (rows of
  // K̂x). A sampled loop is only stable while, roughly,
  //   |Kp|·T²/J + |Kd|·T/J  <  1      (T = control period, J = joint inertia)
  // so the light joints (waist with the arm upright, wrist) need small bounds.
  Vec4 Kp_max = (Vec4() << 60.0, 300.0, 150.0, 15.0).finished();
  Vec4 Kd_max = (Vec4() <<  5.0,  50.0,  15.0,  0.3).finished();
  Vec4 tau_limit = (Vec4() << 150.0, 150.0, 100.0, 50.0).finished();
};

// Adaptive parameters — what save/load stores.
struct MracWeights
{
  MatKx Kx = MatKx::Zero();
  MatKr Kr = MatKr::Zero();
  MatW  W  = MatW::Zero();
};

struct MracOutput
{
  Vec4 tau   = Vec4::Zero();   // clamped torque command
  Vec4 u_ad  = Vec4::Zero();   // −Ŵᵀα
  Vec4 e     = Vec4::Zero();   // q − q_m
  Vec4 ed    = Vec4::Zero();   // q̇ − q̇_m
  Vec4 s     = Vec4::Zero();   // sign(b)·BᵀPe
  Vec4 q_m   = Vec4::Zero();
  Vec4 qd_m  = Vec4::Zero();
  Eigen::Matrix<bool, N_JOINTS, 1> saturated = Eigen::Matrix<bool, N_JOINTS, 1>::Constant(false);
};

// ── Controller core ─────────────────────────────────────────────────────────
class MracCore
{
public:
  MracCore() { configure(MracParams()); }
  explicit MracCore(const MracParams &p) { configure(p); }

  // Sets parameters and resets the adaptive weights to their initial values.
  void configure(const MracParams &p)
  {
    p_   = p;
    ref_ = ReferenceModel(p.wn, p.zeta);

    w0_ = MracWeights();
    for (int i = 0; i < N_JOINTS; i++) {
      w0_.Kx(i, i)            = p.Kp_init(i);
      w0_.Kx(i, N_JOINTS + i) = p.Kd_init(i);
      w0_.Kr(i, i)            = p.Kr_init(i);
    }
    w_ = w0_;

    maskW_ = basisMask();
    if (p.decentralized) {
      maskKx_.setZero();
      maskKr_.setZero();
      for (int i = 0; i < N_JOINTS; i++) {
        maskKx_(i, i) = 1.0;
        maskKx_(i, N_JOINTS + i) = 1.0;
        maskKr_(i, i) = 1.0;
      }
    } else {
      maskKx_.setOnes();
      maskKr_.setOnes();
    }
  }

  void resetReference(const Vec4 &q, const Vec4 &qd) { ref_.reset(q, qd); }

  const ReferenceModel &ref() const { return ref_; }
  const MracParams &params() const { return p_; }
  const MracWeights &weights() const { return w_; }
  void setRobustEnabled(bool on) { p_.robust_enabled = on; }

  // Replaces the weights (e.g. loaded from file). Leakage then pulls toward
  // these values, so they are not eroded while the arm holds still.
  void setWeights(const MracWeights &w)
  {
    w_.Kx = w.Kx.cwiseProduct(maskKx_);
    w_.Kr = w.Kr.cwiseProduct(maskKr_);
    w_.W  = w.W.cwiseProduct(maskW_);
    w0_   = w_;
  }

  // One control step. r is the reference-model input; adapt_Kr is true while
  // a trajectory is executing and false while holding a constant pose.
  MracOutput update(const Vec4 &q, const Vec4 &qd, const Vec4 &r, double dt, bool adapt_Kr)
  {
    MracOutput out;

    ref_.step(r, dt);
    out.q_m  = ref_.q_m();
    out.qd_m = ref_.qd_m();
    out.e    = q - out.q_m;
    out.ed   = qd - out.qd_m;

    Vec8 x;
    x << q, qd;
    const VecB alpha = computeAlpha(q, qd);

    if (p_.robust_enabled) out.u_ad = -(w_.W.transpose() * alpha);
    const Vec4 tau_raw = w_.Kx * x + w_.Kr * r + out.u_ad;
    for (int i = 0; i < N_JOINTS; i++) {
      out.tau(i) = std::clamp(tau_raw(i), -p_.tau_limit(i), p_.tau_limit(i));
      out.saturated(i) = (out.tau(i) != tau_raw(i));
    }

    // s = sign(b)·BᵀPe with B = [0; I]  →  per joint p12·e + p22·ė
    out.s = p_.sign_b * (ref_.P(0, 1) * out.e + ref_.P(1, 1) * out.ed);

    for (int i = 0; i < N_JOINTS; i++) {
      if (out.saturated(i)) continue;          // anti-windup: freeze this joint

      Eigen::Matrix<double, 1, N_STATE> dKx =
          -p_.gamma_Kx(i) * out.s(i) * x.transpose() - p_.sigma * (w_.Kx.row(i) - w0_.Kx.row(i));
      w_.Kx.row(i) += dt * dKx.cwiseProduct(maskKx_.row(i));
      for (int c = 0; c < N_STATE; c++) {
        const double bound = (c < N_JOINTS) ? p_.Kp_max(i) : p_.Kd_max(i);
        w_.Kx(i, c) = std::clamp(w_.Kx(i, c), -bound, bound);
      }

      if (adapt_Kr) {
        Eigen::Matrix<double, 1, N_JOINTS> dKr =
            -p_.gamma_Kr(i) * out.s(i) * r.transpose() - p_.sigma * (w_.Kr.row(i) - w0_.Kr.row(i));
        w_.Kr.row(i) += dt * dKr.cwiseProduct(maskKr_.row(i));
        for (int c = 0; c < N_JOINTS; c++)
          w_.Kr(i, c) = std::clamp(w_.Kr(i, c), -p_.Kr_max, p_.Kr_max);
      }

      if (p_.robust_enabled) {
        VecB dW = p_.gamma_W(i) * out.s(i) * alpha - p_.sigma_W * (w_.W.col(i) - w0_.W.col(i));
        w_.W.col(i) += dt * dW.cwiseProduct(maskW_.col(i));
        const double n = w_.W.col(i).norm();
        if (n > p_.W_max(i)) w_.W.col(i) *= (p_.W_max(i) / n);
      }
    }
    return out;
  }

private:
  MracParams     p_;
  ReferenceModel ref_;
  MracWeights    w_, w0_;
  MatKx          maskKx_ = MatKx::Zero();
  MatKr          maskKr_ = MatKr::Zero();
  MatW           maskW_  = MatW::Zero();
};

}  // namespace mrac_controller
