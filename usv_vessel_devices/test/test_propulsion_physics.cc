#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "usv_vessel_devices/PropulsionPhysics.hh"

using usv_vessel_devices::SubmergedDiskFraction;
using usv_vessel_devices::PropellerSpeed;

TEST(PropulsionPhysics, DiskImmersionIsContinuousAndSymmetric)
{
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(-0.1, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(0.1, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(0.0, 0.1), 0.5);
  EXPECT_NEAR(SubmergedDiskFraction(0.05, 0.1) + SubmergedDiskFraction(-0.05, 0.1), 1.0, 1e-12);
  EXPECT_LT(SubmergedDiskFraction(-0.05, 0.1), SubmergedDiskFraction(0.05, 0.1));
}

TEST(PropulsionPhysics, HorizontalDiskAndInvalidInput)
{
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(-0.001, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(0.001, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(0.0, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(NAN, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(SubmergedDiskFraction(0.0, -1.0), 0.0);
}

TEST(PropulsionPhysics, RotationIsIndependentOfImmersion)
{
  const double speed = PropellerSpeed(100.0, 1000.0, 0.004422, 0.2);
  EXPECT_GT(speed, 0.0);
  EXPECT_DOUBLE_EQ(PropellerSpeed(-100.0, 1000.0, 0.004422, 0.2), -speed);
  EXPECT_DOUBLE_EQ(PropellerSpeed(0.0, 1000.0, 0.004422, 0.2), 0.0);
  EXPECT_DOUBLE_EQ(PropellerSpeed(NAN, 1000.0, 0.004422, 0.2), 0.0);
}
