// Copyright 2026 USV simulation contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef VRX_FLEET_HYDROSTATICS_HH_
#define VRX_FLEET_HYDROSTATICS_HH_

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>
#include <gz/math/Box.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace vrx
{
// Clip six box faces and the waterline cap, then integrate convex tetrahedra.
// Unlike inclusion-exclusion formulas divided by nx*ny*nz, this remains well
// conditioned when roll/pitch are ~1e-19 after a physics update. Those almost
// horizontal cases are normal, not degenerate vessels.
inline std::pair<double, gz::math::Vector3d> SubmergedBox(
  const gz::math::Boxd & box, const gz::math::Planed & plane)
{
  using Vec = gz::math::Vector3d;
  const Vec half = box.Size() * .5;
  const Vec n = plane.Normal();
  const double extent = std::abs(n.X()) * half.X() +
    std::abs(n.Y()) * half.Y() + std::abs(n.Z()) * half.Z();
  if (plane.Offset() <= -extent) return {0., Vec::Zero};
  if (plane.Offset() >= extent) return {box.Volume(), Vec::Zero};
  const std::array<Vec, 8> vertices = {{
    {-half.X(), -half.Y(), -half.Z()}, {half.X(), -half.Y(), -half.Z()},
    {half.X(), half.Y(), -half.Z()}, {-half.X(), half.Y(), -half.Z()},
    {-half.X(), -half.Y(), half.Z()}, {half.X(), -half.Y(), half.Z()},
    {half.X(), half.Y(), half.Z()}, {-half.X(), half.Y(), half.Z()}}};
  const std::array<std::array<int, 4>, 6> indices = {{{0, 1, 2, 3}, {4, 5, 6, 7},
    {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}}};
  std::vector<std::vector<Vec>> faces;
  std::vector<Vec> cap;
  const double tolerance = 1e-10 * std::max({1., half.X(), half.Y(), half.Z()});
  for (const auto & face : indices) {
    std::vector<Vec> clipped;
    Vec previous = vertices[face.back()];
    double dp = plane.Distance(previous);
    for (int index : face) {
      const Vec point = vertices[index];
      const double d = plane.Distance(point);
      if ((d <= 0.) != (dp <= 0.)) {
        const Vec crossing = previous + (point - previous) * (dp / (dp - d));
        clipped.push_back(crossing);
        if (std::none_of(cap.begin(), cap.end(), [&](const Vec & v) {
            return (v - crossing).Length() < tolerance;
          })) cap.push_back(crossing);
      }
      if (d <= 0.) clipped.push_back(point);
      previous = point;
      dp = d;
    }
    if (clipped.size() >= 3) faces.push_back(std::move(clipped));
  }
  if (cap.size() >= 3) {
    Vec center;
    for (const auto & v : cap) center += v;
    center /= cap.size();
    const Vec u = n.Cross(std::abs(n.X()) < .9 ? Vec::UnitX : Vec::UnitY).Normalized();
    const Vec v = n.Cross(u);
    std::sort(cap.begin(), cap.end(), [&](const Vec & a, const Vec & b) {
      return std::atan2((a - center).Dot(v), (a - center).Dot(u)) <
             std::atan2((b - center).Dot(v), (b - center).Dot(u));
    });
    faces.push_back(std::move(cap));
  }
  Vec reference;
  size_t count = 0;
  for (const auto & face : faces) {
    for (const auto & p : face) {reference += p; ++count;}
  }
  if (!count) return {0., Vec::Zero};
  reference /= count;
  double volume = 0.;
  Vec moment;
  for (const auto & face : faces) {
    const Vec a = face[0] - reference;
    for (size_t i = 1; i + 1 < face.size(); ++i) {
      const Vec b = face[i] - reference, c = face[i + 1] - reference;
      const double tetra = std::abs(a.Dot(b.Cross(c))) / 6.;
      volume += tetra;
      moment += (a + b + c) * (tetra / 4.);
    }
  }
  return {std::clamp(volume, 0., box.Volume()),
    volume > 0. ? reference + moment / volume : Vec::Zero};
}

// Archimedes forces at submerged / emerged box centroids. The interface plane
// is transformed into the collision frame, retaining roll/pitch restoration.
inline std::pair<gz::math::Vector3d, gz::math::Vector3d> GradedBoxWrench(
  const gz::math::Boxd & box, const gz::math::Pose3d & collisionPose,
  const gz::math::Vector3d & centerOfMass, const gz::math::Vector3d & gravity,
  double level = 0.0, double waterDensity = 1025.0, double airDensity = 1.225)
{
  const auto normal = collisionPose.Rot().RotateVectorReverse(gz::math::Vector3d::UnitZ);
  const gz::math::Planed plane(normal, level - collisionPose.Pos().Z());
  const double total = box.Volume();
  const auto submerged = SubmergedBox(box, plane);
  const double wet = submerged.first;
  const auto cov = submerged.second;
  const auto waterForce = -gravity * (waterDensity * wet);
  const auto airForce = -gravity * (airDensity * (total - wet));
  const auto waterPoint = collisionPose.Pos() + collisionPose.Rot().RotateVector(cov);
  const auto dryCenter = total - wet > 1e-10 ? -cov * wet / (total - wet) :
    gz::math::Vector3d::Zero;
  const auto airPoint = collisionPose.Pos() + collisionPose.Rot().RotateVector(dryCenter);
  return {waterForce + airForce,
    (waterPoint - centerOfMass).Cross(waterForce) +
    (airPoint - centerOfMass).Cross(airForce)};
}
}  // namespace vrx
#endif
