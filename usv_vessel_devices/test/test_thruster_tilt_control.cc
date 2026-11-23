#include <cmath>
#include <gtest/gtest.h>
#include "usv_vessel_devices/MastControl.hh"
#include "usv_vessel_devices/ThrusterTiltControl.hh"
#include "usv_vessel_devices/ThrusterTiltLimits.hh"

using usv_vessel_devices::TiltThrustEnabled;

TEST(ThrusterTiltControl, HardLimitIsSeventyFiveDegreesAtEveryCommandEntry)
{
  const double maximum = usv_vessel_devices::kMaxThrusterTiltAngle;
  EXPECT_NEAR(maximum, 75.0 * M_PI / 180.0, 1e-15);
  EXPECT_TRUE(usv_vessel_devices::ValidTiltLimit(maximum));
  EXPECT_TRUE(usv_vessel_devices::ValidTiltTarget(0.0, maximum));
  EXPECT_TRUE(usv_vessel_devices::ValidTiltTarget(maximum, maximum));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(maximum + 1e-9, maximum));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(M_PI_2, maximum));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(-1e-9, maximum));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(NAN, maximum));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(INFINITY, maximum));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltLimit(M_PI_2));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltLimit(0.0));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(0.0, M_PI_2));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(0.0, NAN));
  EXPECT_FALSE(usv_vessel_devices::ValidTiltTarget(0.6, 0.5));
}

TEST(ThrusterTiltControl, ThrustRequiresLoweredStoppedReadyFeedback)
{
  EXPECT_TRUE(TiltThrustEnabled(true, 0, 0, 0, 0.005));
  EXPECT_TRUE(TiltThrustEnabled(true, -0.0001, 0, 0, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(false, 0, 0, 0, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(true, 0.01, 0, 0, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(true, 0, 0.01, 0, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(true, NAN, 0, 0, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(true, 0, NAN, 0, 0.005));
}

TEST(ThrusterTiltControl, RaisingInhibitsThrustBeforeAnyMotion)
{
  EXPECT_FALSE(TiltThrustEnabled(true, 0, 0, M_PI_2, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(true, 0, 0, 0.0001, 0.005));
  EXPECT_FALSE(TiltThrustEnabled(true, M_PI_2, 0, 0, 0.005));
}

TEST(ThrusterTiltControl, LoweredLoadTransientsDoNotDropLatchedThrust)
{
  EXPECT_TRUE(TiltThrustEnabled(true, 0.001, -0.01, 0, 0.005, true));
  EXPECT_FALSE(TiltThrustEnabled(true, 0.001, -0.01, 0, 0.005, false));
  EXPECT_FALSE(TiltThrustEnabled(true, 0.006, 0, 0, 0.005, true));
  EXPECT_FALSE(TiltThrustEnabled(true, 0, 0, 0.1, 0.005, true));
  EXPECT_FALSE(TiltThrustEnabled(true, 0, NAN, 0, 0.005, true));
  EXPECT_FALSE(TiltThrustEnabled(false, 0, 0, 0, 0.005, true));
}

TEST(ThrusterTiltControl, LimitsTiltSpeedInBothDirections)
{
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastVelocity(0, M_PI_2, 0.25, 0.005, 0.001), 0.25);
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastVelocity(M_PI_2, 0, 0.25, 0.005, 0.001), -0.25);
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastVelocity(0, 0, 0.25, 0.005, 0.001), 0);
}

TEST(ThrusterTiltControl, PausedOrInvalidFeedbackCannotMoveJoint)
{
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastVelocity(NAN, 1, 0.25, 0.005, 0.001), 0);
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastVelocity(0, 1, 0.25, 0.005, 0), 0);
  EXPECT_DOUBLE_EQ(usv_vessel_devices::MastVelocity(0, 1, 0.25, 0.005, NAN), 0);
}
