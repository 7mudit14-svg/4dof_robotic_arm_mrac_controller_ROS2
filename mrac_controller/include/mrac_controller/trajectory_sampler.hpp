/*
 * trajectory_sampler.hpp — cubic Hermite sampling of a JointTrajectory.
 *
 * MoveIt supplies waypoint positions and velocities; between waypoints the
 * position is a cubic Hermite segment. This returns the desired position,
 * velocity and acceleration (q_d, q̇_d, q̈_d) of one joint column at time t.
 */
#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <cstddef>

namespace mrac_controller
{

struct Sample { double q = 0.0, dq = 0.0, ddq = 0.0; };

inline double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint &p)
{
  return static_cast<double>(p.time_from_start.sec) + 1e-9 * static_cast<double>(p.time_from_start.nanosec);
}

inline double trajectoryEndTime(const trajectory_msgs::msg::JointTrajectory &traj)
{
  return traj.points.empty() ? 0.0 : pointTime(traj.points.back());
}

// Samples column `col` of the trajectory at time t [s from trajectory start].
// Before the first point it returns the first point; after the last point it
// returns the last point at rest. Returns false only for an unusable
// trajectory (no points, or `col` outside a point's positions).
inline bool sampleTraj(const trajectory_msgs::msg::JointTrajectory &traj, double t, std::size_t col, Sample &s)
{
  const auto &pts = traj.points;
  if (pts.empty()) return false;
  for (const auto &p : pts)
    if (col >= p.positions.size()) return false;

  if (pts.size() == 1 || t >= pointTime(pts.back())) {
    s.q = pts.back().positions[col];
    s.dq = 0.0;
    s.ddq = 0.0;
    return true;
  }
  if (t <= pointTime(pts.front())) {
    s.q = pts.front().positions[col];
    s.dq = 0.0;
    s.ddq = 0.0;
    return true;
  }

  std::size_t k = pts.size() - 2;
  for (std::size_t i = 0; i + 1 < pts.size(); i++) {
    if (t < pointTime(pts[i + 1])) { k = i; break; }
  }

  const double t0  = pointTime(pts[k]);
  const double t1  = pointTime(pts[k + 1]);
  const double seg = std::max(t1 - t0, 1e-6);
  const double u   = std::clamp((t - t0) / seg, 0.0, 1.0);

  const double p0 = pts[k].positions[col];
  const double p1 = pts[k + 1].positions[col];
  const double v0 = col < pts[k].velocities.size()     ? pts[k].velocities[col]     : 0.0;
  const double v1 = col < pts[k + 1].velocities.size() ? pts[k + 1].velocities[col] : 0.0;

  const double u2 = u * u, u3 = u2 * u;

  s.q = (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * seg * v0
      + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * seg * v1;

  s.dq = ((6 * u2 - 6 * u) * p0 + (3 * u2 - 4 * u + 1) * seg * v0
        + (-6 * u2 + 6 * u) * p1 + (3 * u2 - 2 * u) * seg * v1) / seg;

  s.ddq = ((12 * u - 6) * p0 + (6 * u - 4) * seg * v0
         + (-12 * u + 6) * p1 + (6 * u - 2) * seg * v1) / (seg * seg);

  return true;
}

}  // namespace mrac_controller
