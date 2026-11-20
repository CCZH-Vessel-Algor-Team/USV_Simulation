/* Copyright (C) 2026. Licensed under the Apache License, Version 2.0. */
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "CurrentAdvection.hh"

TEST(CurrentAdvection, StartsAtRest)
{
  vrx::CurrentAdvection phase;
  phase.Advance(60.0);
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[0]);
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[1]);
}

TEST(CurrentAdvection, MetresPerSecondBecomeTextureRepetitions)
{
  vrx::CurrentAdvection phase;
  // CCS has 900 repeats over 36000 m: 1 m/s is 0.025 UV/s.
  EXPECT_TRUE(phase.SetVelocity(1.0 * 900.0 / 36000.0, 0.0));
  phase.Advance(4.0);
  EXPECT_NEAR(0.1, phase.Offset()[0], 1e-12);
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[1]);
}

TEST(CurrentAdvection, PositiveAndNegativeAxes)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(-0.1, 0.2);
  phase.Advance(2.0);
  EXPECT_NEAR(0.8, phase.Offset()[0], 1e-12);
  EXPECT_NEAR(0.4, phase.Offset()[1], 1e-12);
}

TEST(CurrentAdvection, CcsNorthUsesFlippedTextureV)
{
  vrx::CurrentAdvection phase;
  // Renderer V decreases along local/world +Y for the CCS COLLADA mesh.
  phase.SetVelocity(0.0, -1.0 * 900.0 / 36000.0);
  phase.Advance(4.0);
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[0]);
  EXPECT_NEAR(0.9, phase.Offset()[1], 1e-12);
}

TEST(CurrentAdvection, TurningDoesNotResetOrJumpPhase)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.025, 0.0);
  phase.Advance(10.0);
  phase.SetVelocity(0.0, 0.05);
  EXPECT_NEAR(0.25, phase.Offset()[0], 1e-12);
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[1]);
  phase.Advance(2.0);
  EXPECT_NEAR(0.25, phase.Offset()[0], 1e-12);
  EXPECT_NEAR(0.1, phase.Offset()[1], 1e-12);
}

TEST(CurrentAdvection, ReversalRetracesTheSameTexture)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.03, -0.02);
  phase.Advance(7.0);
  phase.SetVelocity(-0.03, 0.02);
  phase.Advance(7.0);
  // Phase 1 and phase 0 are identical on a repeating texture.
  const auto periodicError = [](double _v)
  {
    return std::min(std::abs(_v), std::abs(_v - 1.0));
  };
  EXPECT_NEAR(0.0, periodicError(phase.Offset()[0]), 1e-12);
  EXPECT_NEAR(0.0, periodicError(phase.Offset()[1]), 1e-12);
}

TEST(CurrentAdvection, ZeroCurrentStopsWithoutResetting)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.025, 0.05);
  phase.Advance(3.0);
  const auto before = phase.Offset();
  phase.SetVelocity(0.0, 0.0);
  phase.Advance(1000.0);
  EXPECT_EQ(before, phase.Offset());
}

TEST(CurrentAdvection, ZeroDtKeepsPhaseWhilePaused)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.025, 0.05);
  phase.Advance(3.0);
  const auto before = phase.Offset();
  phase.Advance(0.0);
  EXPECT_EQ(before, phase.Offset());
}

TEST(CurrentAdvection, AnExplicitSingleStepAdvancesBySimulationDt)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.025, 0.0);
  phase.Advance(0.004);
  EXPECT_NEAR(0.0001, phase.Offset()[0], 1e-12);
}

TEST(CurrentAdvection, ResetClearsPhaseButKeepsCurrent)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.025, -0.05);
  phase.Advance(3.0);
  phase.ResetPhase();
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[0]);
  EXPECT_DOUBLE_EQ(0.0, phase.Offset()[1]);
  phase.Advance(1.0);
  EXPECT_NEAR(0.025, phase.Offset()[0], 1e-12);
  EXPECT_NEAR(0.95, phase.Offset()[1], 1e-12);
}

TEST(CurrentAdvection, IntegrationDoesNotDependOnFrameRate)
{
  vrx::CurrentAdvection slow, fast;
  slow.SetVelocity(0.025, 0.017);
  fast.SetVelocity(0.025, 0.017);
  slow.Advance(4.0);
  for (int i = 0; i < 1000; ++i)
    fast.Advance(0.004);
  EXPECT_NEAR(slow.Offset()[0], fast.Offset()[0], 1e-12);
  EXPECT_NEAR(slow.Offset()[1], fast.Offset()[1], 1e-12);
}

TEST(CurrentAdvection, LongRunsKeepUniformsBounded)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.031, -0.027);
  for (int i = 0; i < 10000; ++i)
    phase.Advance(100.0);
  for (const auto value : phase.Offset())
  {
    EXPECT_GE(value, 0.0);
    EXPECT_LT(value, 1.0);
  }
}

TEST(CurrentAdvection, RejectsNonFiniteVelocityAndDt)
{
  vrx::CurrentAdvection phase;
  phase.SetVelocity(0.025, 0.05);
  EXPECT_FALSE(phase.SetVelocity(std::numeric_limits<double>::infinity(), 0));
  EXPECT_FALSE(phase.SetVelocity(0, std::numeric_limits<double>::quiet_NaN()));
  phase.Advance(1.0);
  const auto before = phase.Offset();
  EXPECT_NEAR(0.025, before[0], 1e-12);
  EXPECT_NEAR(0.05, before[1], 1e-12);
  phase.Advance(-10.0);
  phase.Advance(std::numeric_limits<double>::infinity());
  phase.Advance(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(before, phase.Offset());
}
