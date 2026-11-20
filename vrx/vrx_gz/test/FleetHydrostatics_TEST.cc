#include <gtest/gtest.h>
#include "FleetHydrostatics.hh"

TEST(FleetHydrostatics, HalfSubmergedBox)
{
  const auto result = vrx::GradedBoxWrench(gz::math::Boxd(4, 2, 2),
    gz::math::Pose3d::Zero, gz::math::Vector3d::Zero, {0, 0, -9.81});
  EXPECT_NEAR(result.first.Z(), 8 * 9.81 * (1025 + 1.225), 1e-6);
  EXPECT_NEAR(result.second.Length(), 0.0, 1e-8);
}

TEST(FleetHydrostatics, DryAndFullyWet)
{
  const gz::math::Boxd box(4, 2, 2);
  const auto dry = vrx::GradedBoxWrench(box, {0, 0, 10, 0, 0, 0}, {0, 0, 10}, {0, 0, -9.81});
  const auto wet = vrx::GradedBoxWrench(box, {0, 0, -10, 0, 0, 0}, {0, 0, -10}, {0, 0, -9.81});
  EXPECT_NEAR(dry.first.Z(), 16 * 1.225 * 9.81, 1e-6);
  EXPECT_NEAR(wet.first.Z(), 16 * 1025 * 9.81, 1e-6);
}

TEST(FleetHydrostatics, InclinedWideHullHasRestoringMoment)
{
  const gz::math::Boxd box(20, 10, 6);
  const auto result = vrx::GradedBoxWrench(box, {0, 0, 0, .05, 0, 0}, {0, 0, -2}, {0, 0, -9.81});
  EXPECT_TRUE(result.first.IsFinite());
  EXPECT_TRUE(result.second.IsFinite());
  EXPECT_LT(result.second.X(), 0.0);
}

TEST(FleetHydrostatics, AlmostHorizontalContainerShipRetainsReferenceDisplacement)
{
  const gz::math::Boxd hull(359.1, 49.3, 30.);
  for (double roll : {0., 1e-19, 1e-12, 1e-8, -1e-8}) {
    const auto result = vrx::GradedBoxWrench(hull, {0, 0, -1.5, roll, -roll, .7},
      {0, 0, -9.9}, {0, 0, -9.81});
    EXPECT_NEAR(result.first.Z(), 299705416.156125 * 9.81, .1) << roll;
    EXPECT_TRUE(result.second.IsFinite());
  }
}

TEST(FleetHydrostatics, ClippedVolumeAndCentroidMatchAnalyticHorizontalSlices)
{
  const gz::math::Boxd hull(4, 2, 2);
  for (double level : {-.99, -.5, 0., .1, .9, .99}) {
    const auto result = vrx::SubmergedBox(hull, gz::math::Planed({0, 0, 1}, level));
    EXPECT_NEAR(result.first, 8 * (level + 1), 1e-10);
    EXPECT_NEAR(result.second.Z(), (level - 1) / 2, 1e-10);
    EXPECT_NEAR(result.second.X(), 0., 1e-10);
    EXPECT_NEAR(result.second.Y(), 0., 1e-10);
  }
}

TEST(FleetHydrostatics, ObliqueComplementConservesTotalVolumeAndFirstMoment)
{
  const gz::math::Boxd hull(399, 58, 30);
  for (const auto & normal : {gz::math::Vector3d(.1, .2, 1).Normalized(),
      gz::math::Vector3d(1e-18, -.1, 1).Normalized(), gz::math::Vector3d(1, 1, 1).Normalized()}) {
    const auto wet = vrx::SubmergedBox(hull, gz::math::Planed(normal, 3.));
    const auto dry = vrx::SubmergedBox(hull, gz::math::Planed(-normal, -3.));
    EXPECT_NEAR(wet.first + dry.first, hull.Volume(), 1e-6);
    EXPECT_LT((wet.second * wet.first + dry.second * dry.first).Length(), 1e-5);
  }
}
