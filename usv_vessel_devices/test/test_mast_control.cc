#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "usv_vessel_devices/MastControl.hh"

using usv_vessel_devices::MastVelocity;
using usv_vessel_devices::ValidTarget;

TEST(MastControl, RejectsNonfiniteAndOutOfRangeTargets)
{
  EXPECT_TRUE(ValidTarget(0, M_PI_2));
  EXPECT_TRUE(ValidTarget(M_PI_2, M_PI_2));
  EXPECT_FALSE(ValidTarget(-0.001, M_PI_2));
  EXPECT_FALSE(ValidTarget(M_PI, M_PI_2));
  EXPECT_FALSE(ValidTarget(std::numeric_limits<double>::quiet_NaN(), M_PI_2));
  EXPECT_FALSE(ValidTarget(std::numeric_limits<double>::infinity(), M_PI_2));
}

TEST(MastControl, LimitsSpeedAndAvoidsOneStepOvershoot)
{
  EXPECT_DOUBLE_EQ(MastVelocity(0, 1.5, 0.3, 0.005, 0.001), 0.3);
  EXPECT_DOUBLE_EQ(MastVelocity(1.5, 0, 0.3, 0.005, 0.001), -0.3);
  EXPECT_NEAR(MastVelocity(0.999, 1.0, 0.3, 0.005, 0.001), 0.004, 1e-12);
  EXPECT_DOUBLE_EQ(MastVelocity(0.9999, 1.0, 0.3, 0.005, 0.001), 0);
  EXPECT_LE(MastVelocity(0.98, 1.0, 0.3, 0.005, 2.0), 0.01 + 1e-12);
  EXPECT_DOUBLE_EQ(MastVelocity(0, 1.0, 0.3, 0.005, 0), 0);
  EXPECT_DOUBLE_EQ(MastVelocity(NAN, 1.0, 0.3, 0.005, 0.001), 0);
}

TEST(MastControl, EffortCanRaiseMastAndHoldAgainstGravity)
{
  double integral = -5.0;
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastEffort(0, 0, 0.001, integral), -5.0);
  EXPECT_LT(usv_vessel_devices::MastEffort(-0.3, 0, 0.001, integral), -5.0);
  EXPECT_LE(std::abs(usv_vessel_devices::MastEffort(100, -100, 1, integral)), 100.0);
  EXPECT_LE(std::abs(integral), 100.0);
}
