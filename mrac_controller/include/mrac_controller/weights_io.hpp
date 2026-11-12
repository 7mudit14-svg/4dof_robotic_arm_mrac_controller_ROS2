/*
 * weights_io.hpp — save / load the adaptive weights (K̂x, K̂r, Ŵ) as YAML.
 */
#pragma once

#include "mrac_controller/mrac_core.hpp"

#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace mrac_controller
{

namespace detail
{
template <typename Derived>
YAML::Node matrixToYaml(const Eigen::MatrixBase<Derived> &m)
{
  YAML::Node rows(YAML::NodeType::Sequence);
  for (Eigen::Index r = 0; r < m.rows(); r++) {
    YAML::Node row(YAML::NodeType::Sequence);
    row.SetStyle(YAML::EmitterStyle::Flow);
    for (Eigen::Index c = 0; c < m.cols(); c++) row.push_back(m(r, c));
    rows.push_back(row);
  }
  return rows;
}

template <typename Derived>
bool yamlToMatrix(const YAML::Node &n, Eigen::MatrixBase<Derived> &m)
{
  if (!n.IsSequence() || static_cast<Eigen::Index>(n.size()) != m.rows()) return false;
  for (Eigen::Index r = 0; r < m.rows(); r++) {
    const YAML::Node row = n[static_cast<std::size_t>(r)];
    if (!row.IsSequence() || static_cast<Eigen::Index>(row.size()) != m.cols()) return false;
    for (Eigen::Index c = 0; c < m.cols(); c++) {
      const double v = row[static_cast<std::size_t>(c)].as<double>();
      if (!std::isfinite(v)) return false;
      m(r, c) = v;
    }
  }
  return true;
}
}  // namespace detail

// Returns true on success; on failure `error` says why.
inline bool saveWeights(const std::string &path, const MracWeights &w, std::string &error)
{
  try {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());

    YAML::Node root;
    root["n_joints"] = N_JOINTS;
    root["n_basis"]  = N_BASIS;
    root["Kx"] = detail::matrixToYaml(w.Kx);   // 4 rows × 8: [Kp | Kd]
    root["Kr"] = detail::matrixToYaml(w.Kr);   // 4 × 4
    root["W"]  = detail::matrixToYaml(w.W);    // n_basis rows × 4 joints

    std::ofstream out(path);
    if (!out) { error = "cannot open " + path + " for writing"; return false; }
    out << "# MRAC adaptive weights. Kx rows = joints, columns = [q1..q4, qd1..qd4].\n";
    out << "# W rows = basis terms [1, sin(q2), sin(q2+q3), tanh(qd1)..tanh(qd4)], columns = joints.\n";
    out << root << "\n";
    return static_cast<bool>(out);
  } catch (const std::exception &ex) {
    error = ex.what();
    return false;
  }
}

// Leaves `w` untouched unless the whole file is valid.
inline bool loadWeights(const std::string &path, MracWeights &w, std::string &error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root["n_joints"] || !root["n_basis"] ||
        root["n_joints"].as<int>() != N_JOINTS || root["n_basis"].as<int>() != N_BASIS) {
      error = "weights file does not match this controller (n_joints / n_basis)";
      return false;
    }
    MracWeights tmp;
    if (!detail::yamlToMatrix(root["Kx"], tmp.Kx) ||
        !detail::yamlToMatrix(root["Kr"], tmp.Kr) ||
        !detail::yamlToMatrix(root["W"], tmp.W)) {
      error = "weights file has a matrix of the wrong size or a non-finite value";
      return false;
    }
    w = tmp;
    return true;
  } catch (const std::exception &ex) {
    error = ex.what();
    return false;
  }
}

}  // namespace mrac_controller
