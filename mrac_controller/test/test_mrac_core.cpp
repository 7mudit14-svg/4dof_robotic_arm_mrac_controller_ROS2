#include <gtest/gtest.h>

#include "mrac_controller/mrac_core.hpp"
#include "mrac_controller/trajectory_sampler.hpp"
#include "mrac_controller/weights_io.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace mrac_controller;

// ── Reference model ─────────────────────────────────────────────────────────
TEST(ReferenceModel, LyapunovEquation)
{
  for (double wn : {2.0, 3.0, 8.0}) {
    for (double zeta : {0.5, 0.76, 1.0}) {
      ReferenceModel m(wn, zeta);
      const Eigen::Matrix2d lhs = m.P * m.Am + m.Am.transpose() * m.P;
      EXPECT_TRUE(lhs.isApprox(-Eigen::Matrix2d::Identity(), 1e-9));
      EXPECT_GT(m.P.determinant(), 0.0);
      EXPECT_GT(m.P(0, 0), 0.0);
    }
  }
}

TEST(ReferenceModel, StepSettlesAtReference)
{
  ReferenceModel m(3.0, 0.76);
  m.reset(Vec4::Zero(), Vec4::Zero());
  const Vec4 r = (Vec4() << 0.5, -1.0, 2.0, 0.1).finished();
  for (int k = 0; k < 1000; k++) m.step(r, 0.01);   // 10 s at 100 Hz
  EXPECT_TRUE(m.q_m().isApprox(r, 1e-4));
  EXPECT_LT(m.qd_m().norm(), 1e-4);
}

TEST(ReferenceModel, PrecompensationRemovesLag)
{
  ReferenceModel m(3.0, 0.76);
  const double w = 0.8, dt = 0.001;
  auto qd_of = [&](double t, Vec4 &q, Vec4 &v, Vec4 &a) {
    q = Vec4::Constant(0.5 * std::sin(w * t));
    v = Vec4::Constant(0.5 * w * std::cos(w * t));
    a = Vec4::Constant(-0.5 * w * w * std::sin(w * t));
  };
  Vec4 q, v, a;
  qd_of(0.0, q, v, a);
  m.reset(q, v);
  double worst = 0.0;
  for (int k = 0; k < 20000; k++) {
    qd_of(k * dt, q, v, a);
    m.step(m.precompensate(q, v, a), dt);
    qd_of((k + 1) * dt, q, v, a);
    worst = std::max(worst, (m.q_m() - q).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(worst, 2e-3);   // without pre-compensation the lag error is ~0.2 rad
}

// ── Trajectory sampler ──────────────────────────────────────────────────────
static trajectory_msgs::msg::JointTrajectory makeTraj()
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint1", "joint2"};
  const double times[3] = {0.0, 1.0, 2.5};
  const double pos[3][2] = {{0.0, 1.0}, {0.5, 0.5}, {1.0, 0.2}};
  const double vel[3][2] = {{0.0, 0.0}, {0.4, -0.3}, {0.0, 0.0}};
  for (int k = 0; k < 3; k++) {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {pos[k][0], pos[k][1]};
    p.velocities = {vel[k][0], vel[k][1]};
    p.time_from_start.sec = static_cast<int32_t>(times[k]);
    p.time_from_start.nanosec = static_cast<uint32_t>((times[k] - std::floor(times[k])) * 1e9);
    traj.points.push_back(p);
  }
  return traj;
}

TEST(Sampler, HitsWaypointsAndEnd)
{
  const auto traj = makeTraj();
  Sample s;
  ASSERT_TRUE(sampleTraj(traj, 0.0, 0, s));
  EXPECT_NEAR(s.q, 0.0, 1e-12);
  ASSERT_TRUE(sampleTraj(traj, 1.0, 1, s));
  EXPECT_NEAR(s.q, 0.5, 1e-9);
  EXPECT_NEAR(s.dq, -0.3, 1e-9);
  // At and after the end: final point, at rest (this used to return zeros).
  for (double t : {2.5, 2.6, 3.5, 100.0}) {
    ASSERT_TRUE(sampleTraj(traj, t, 0, s));
    EXPECT_DOUBLE_EQ(s.q, 1.0);
    EXPECT_DOUBLE_EQ(s.dq, 0.0);
    EXPECT_DOUBLE_EQ(s.ddq, 0.0);
  }
  EXPECT_DOUBLE_EQ(trajectoryEndTime(traj), 2.5);
}

TEST(Sampler, DerivativesMatchFiniteDifferences)
{
  const auto traj = makeTraj();
  const double h = 1e-5;
  for (double t : {0.2, 0.7, 1.3, 2.1}) {
    Sample a, b, c;
    ASSERT_TRUE(sampleTraj(traj, t - h, 0, a));
    ASSERT_TRUE(sampleTraj(traj, t, 0, b));
    ASSERT_TRUE(sampleTraj(traj, t + h, 0, c));
    EXPECT_NEAR(b.dq, (c.q - a.q) / (2 * h), 1e-6);
    EXPECT_NEAR(b.ddq, (c.dq - a.dq) / (2 * h), 1e-5);
  }
}

TEST(Sampler, RejectsBadInput)
{
  trajectory_msgs::msg::JointTrajectory empty;
  Sample s;
  EXPECT_FALSE(sampleTraj(empty, 0.0, 0, s));
  const auto traj = makeTraj();
  EXPECT_FALSE(sampleTraj(traj, 0.5, 2, s));   // column out of range
}

// ── Core: basis and masks ───────────────────────────────────────────────────
TEST(Core, BasisAndMask)
{
  Vec4 q, qd;
  q << 0.3, 0.7, 1.1, -0.4;
  qd << 0.1, -0.2, 0.3, 0.4;
  const VecB a = computeAlpha(q, qd);
  EXPECT_DOUBLE_EQ(a(0), 1.0);
  EXPECT_NEAR(a(1), std::sin(0.7), 1e-12);
  EXPECT_NEAR(a(2), std::sin(0.7 + 1.1), 1e-12);
  EXPECT_NEAR(a(4), std::tanh(-0.2), 1e-12);
  EXPECT_NEAR(a(6), std::tanh(0.4), 1e-12);

  const MatW m = basisMask();
  EXPECT_EQ(m.col(0).sum(), 2.0);   // joint1: offset + friction
  EXPECT_EQ(m.col(1).sum(), 4.0);   // joint2: offset + two gravity terms + friction
  EXPECT_EQ(m.col(2).sum(), 3.0);
  EXPECT_EQ(m.col(3).sum(), 2.0);
}

TEST(Core, DecentralizedKeepsCrossTermsZero)
{
  MracParams p;
  p.sigma = 0.01;
  p.sigma_W = 0.01;
  MracCore core(p);
  Vec4 q, qd, r;
  q << 0.3, 0.7, 1.1, -0.4;
  qd << 0.5, -0.2, 0.3, 0.4;
  r << 0.0, 1.0, 0.5, 0.2;
  core.resetReference(Vec4::Zero(), Vec4::Zero());
  for (int k = 0; k < 500; k++) core.update(q, qd, r, 0.002, true);
  const auto &w = core.weights();
  for (int i = 0; i < N_JOINTS; i++) {
    for (int c = 0; c < N_STATE; c++) {
      if (c != i && c != N_JOINTS + i) { EXPECT_EQ(w.Kx(i, c), 0.0); }
    }
  }
  EXPECT_EQ(w.Kr(0, 1), 0.0);
  EXPECT_EQ(w.W(1, 0), 0.0);        // sin(q2) must not act on joint1
  EXPECT_NE(w.W(1, 1), 0.0);        // …but does adapt on joint2
  EXPECT_NE(w.Kx(1, N_JOINTS + 1), p.Kd_init(1));   // velocity gain adapts
}

TEST(Core, SaturationClampsAndFreezesAdaptation)
{
  MracParams p;
  p.Kp_init = Vec4::Constant(-200.0);
  p.Kr_init = Vec4::Constant(0.0);
  MracCore core(p);
  const Vec4 q = Vec4::Constant(2.0);     // −400 Nm requested on every joint
  core.resetReference(Vec4::Zero(), Vec4::Zero());
  const MracWeights before = core.weights();
  const MracOutput out = core.update(q, Vec4::Zero(), Vec4::Zero(), 0.002, true);
  for (int i = 0; i < N_JOINTS; i++) {
    EXPECT_TRUE(out.saturated(i));
    EXPECT_DOUBLE_EQ(out.tau(i), -p.tau_limit(i));
  }
  EXPECT_TRUE(core.weights().Kx.isApprox(before.Kx));
  EXPECT_TRUE(core.weights().W.isApprox(before.W));
}

// ── Core: closed loop on a gravity-loaded arm model ─────────────────────────
// Joints are independent inertias with viscous + Coulomb friction; joints 2
// and 3 carry the gravity terms of two parallel-axis links that stand upright
// at q = 0 (so gravity pushes them away from zero, as on the real arm).
struct Plant
{
  // Inertias as on the real arm: the waist (arm upright) and the wrist are very light.
  Vec4 J = (Vec4() << 0.012, 1.25, 0.25, 0.0017).finished();
  Vec4 d = (Vec4() << 1.0, 0.8, 0.6, 0.2).finished();
  Vec4 fc = (Vec4() << 0.3, 0.25, 0.15, 0.08).finished();
  double ga = 11.0, gb = 4.0;
  Vec4 q = Vec4::Zero(), qd = Vec4::Zero();

  // The torque is held over the control period (as on the real system) while
  // the physics is integrated with a 10× finer step.
  void step(const Vec4 &tau, double dt)
  {
    const int sub = 10;
    const double h = dt / sub;
    for (int k = 0; k < sub; k++) {
      Vec4 g = Vec4::Zero();
      g(1) = ga * std::sin(q(1)) + gb * std::sin(q(1) + q(2));
      g(2) = gb * std::sin(q(1) + q(2));
      for (int i = 0; i < N_JOINTS; i++) {
        const double qdd = (tau(i) + g(i) - d(i) * qd(i) - fc(i) * std::tanh(qd(i) / 0.05)) / J(i);
        qd(i) += qdd * h;
        q(i) += qd(i) * h;
      }
    }
  }
};

static void desired(double t, Vec4 &q, Vec4 &v, Vec4 &a)
{
  const double A[4] = {0.8, 0.5, 0.9, 0.7};
  const double O[4] = {0.0, 0.6, 1.1, 0.2};
  const double w[4] = {0.31, 0.23, 0.37, 0.41};
  for (int i = 0; i < 4; i++) {
    q(i) = O[i] + A[i] * std::sin(w[i] * t);
    v(i) = A[i] * w[i] * std::cos(w[i] * t);
    a(i) = -A[i] * w[i] * w[i] * std::sin(w[i] * t);
  }
}

// Runs the loop for `seconds`; returns the per-joint RMS of q − q_d over the
// last `window` seconds.
static Vec4 runLoop(MracCore &core, Plant &plant, double t0, double seconds, double window, double dt,
                    bool *finite = nullptr)
{
  Vec4 sum = Vec4::Zero();
  int n = 0;
  const int steps = static_cast<int>(seconds / dt);
  for (int k = 0; k < steps; k++) {
    const double t = t0 + k * dt;
    Vec4 q_d, v_d, a_d;
    desired(t, q_d, v_d, a_d);
    const Vec4 r = core.ref().precompensate(q_d, v_d, a_d);
    const MracOutput out = core.update(plant.q, plant.qd, r, dt, true);
    plant.step(out.tau, dt);
    if (finite && !(out.tau.allFinite() && plant.q.allFinite())) *finite = false;
    if (t - t0 >= seconds - window) {
      sum += (plant.q - q_d).cwiseAbs2();
      n++;
    }
  }
  return (sum / std::max(n, 1)).cwiseSqrt();
}

static MracParams testParams()
{
  MracParams p;
  p.gamma_Kx = Vec4::Constant(200.0);
  p.gamma_Kr = Vec4::Constant(200.0);
  p.gamma_W  = Vec4::Constant(400.0);
  return p;
}

TEST(ClosedLoop, TracksAndImprovesWithAdaptation)
{
  for (double dt : {0.001, 0.0005}) {        // 1 kHz (design rate) and 2 kHz
    MracCore core(testParams());
    Plant plant;
    Vec4 q_d, v_d, a_d;
    desired(0.0, q_d, v_d, a_d);
    plant.q = q_d;
    plant.qd = v_d;
    core.resetReference(plant.q, plant.qd);

    bool finite = true;
    const Vec4 early = runLoop(core, plant, 0.0, 20.0, 20.0, dt, &finite);
    const Vec4 late  = runLoop(core, plant, 20.0, 160.0, 20.0, dt, &finite);
    std::printf("dt=%.3f  early RMS [%.4f %.4f %.4f %.4f]  late RMS [%.4f %.4f %.4f %.4f]\n", dt,
                early(0), early(1), early(2), early(3), late(0), late(1), late(2), late(3));
    const auto &w = core.weights();
    std::printf("          W joint2 gravity [%.2f %.2f] (plant 11, 4)  joint3 [%.2f] (plant 4)\n",
                w.W(1, 1), w.W(2, 1), w.W(2, 2));

    EXPECT_TRUE(finite);
    EXPECT_LT(late.maxCoeff(), 0.02);
    EXPECT_LT(late(1), early(1));
    EXPECT_LT(late(2), early(2));
  }
}

TEST(ClosedLoop, LoadedWeightsStartWhereTrainingEnded)
{
  const double dt = 0.001;
  MracCore trained(testParams());
  Plant plant;
  Vec4 q_d, v_d, a_d;
  desired(0.0, q_d, v_d, a_d);
  plant.q = q_d;
  plant.qd = v_d;
  trained.resetReference(plant.q, plant.qd);
  const Vec4 fresh = runLoop(trained, plant, 0.0, 20.0, 20.0, dt);
  runLoop(trained, plant, 20.0, 160.0, 20.0, dt);

  // Restart from the same initial state with the trained weights loaded.
  MracCore reloaded(testParams());
  reloaded.setWeights(trained.weights());
  Plant plant2;
  plant2.q = q_d;
  plant2.qd = v_d;
  reloaded.resetReference(plant2.q, plant2.qd);
  const Vec4 warm = runLoop(reloaded, plant2, 0.0, 20.0, 20.0, dt);

  std::printf("first 20 s RMS  fresh [%.4f %.4f %.4f %.4f]  with loaded weights [%.4f %.4f %.4f %.4f]\n",
              fresh(0), fresh(1), fresh(2), fresh(3), warm(0), warm(1), warm(2), warm(3));
  // The gravity-loaded joints must start clearly better than an untrained run.
  EXPECT_LT(warm(1), 0.7 * fresh(1));
  EXPECT_LT(warm(2), 0.7 * fresh(2));
}

// ── Save / load ─────────────────────────────────────────────────────────────
TEST(WeightsIo, RoundTripAndRejection)
{
  const std::string dir = (std::filesystem::temp_directory_path() / "mrac_weights_test").string();
  const std::string path = dir + "/w.yaml";

  MracWeights w;
  w.Kx.setRandom();
  w.Kr.setRandom();
  w.W.setRandom();

  std::string err;
  ASSERT_TRUE(saveWeights(path, w, err)) << err;

  MracWeights r;
  ASSERT_TRUE(loadWeights(path, r, err)) << err;
  EXPECT_TRUE(r.Kx.isApprox(w.Kx, 1e-12));
  EXPECT_TRUE(r.Kr.isApprox(w.Kr, 1e-12));
  EXPECT_TRUE(r.W.isApprox(w.W, 1e-12));

  // Missing file and wrong basis size are rejected and leave the target alone.
  MracWeights untouched = r;
  EXPECT_FALSE(loadWeights(dir + "/missing.yaml", r, err));
  {
    std::ofstream bad(dir + "/bad.yaml");
    bad << "n_joints: 4\nn_basis: 22\nKx: []\nKr: []\nW: []\n";
  }
  EXPECT_FALSE(loadWeights(dir + "/bad.yaml", r, err));
  EXPECT_TRUE(r.W.isApprox(untouched.W));

  std::filesystem::remove_all(dir);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
