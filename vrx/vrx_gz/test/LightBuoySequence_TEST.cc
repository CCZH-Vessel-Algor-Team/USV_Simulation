#include <chrono>
#include <gtest/gtest.h>

#include "LightBuoySequence.hh"

using vrx::LightBuoySequence;
using Time = LightBuoySequence::Time;

TEST(LightBuoySequence, LateSpawnDoesNotReplayTimeSinceWorldStart)
{
  LightBuoySequence sequence;
  sequence.SetTime(Time(600.0));
  ASSERT_EQ(sequence.Next(), 0u);
  // A newly added buoy must hold its first color for a full second, even
  // though the world has already been running for ten minutes.
  for (int frame = 1; frame < 60; ++frame)
  {
    sequence.SetTime(Time(600.0 + frame / 60.0));
    EXPECT_FALSE(sequence.Next());
  }
  sequence.SetTime(Time(601.0));
  EXPECT_EQ(sequence.Next(), 1u);
}

TEST(LightBuoySequence, KeepsTheFiveStepColorPattern)
{
  LightBuoySequence sequence;
  for (int second = 0; second < 15; ++second)
  {
    sequence.SetTime(Time(200.0 + second));
    EXPECT_EQ(sequence.Next(), static_cast<std::size_t>(second % 5));
  }
}

TEST(LightBuoySequence, DelayedSceneStartsWithTheFirstColor)
{
  LightBuoySequence sequence;
  sequence.SetTime(Time(600.0));
  // No Next() calls until the render scene has provided the buoy visuals.
  sequence.SetTime(Time(610.0));
  EXPECT_EQ(sequence.Next(), 0u);
  sequence.SetTime(Time(610.01));
  EXPECT_FALSE(sequence.Next());
}

TEST(LightBuoySequence, RenderStallDoesNotProduceABurstOfColors)
{
  LightBuoySequence sequence;
  sequence.SetTime(Time(600.0));
  EXPECT_EQ(sequence.Next(), 0u);
  sequence.SetTime(Time(620.0));
  EXPECT_EQ(sequence.Next(), 1u);
  for (int frame = 1; frame < 60; ++frame)
  {
    sequence.SetTime(Time(620.0 + frame / 60.0));
    EXPECT_FALSE(sequence.Next());
  }
  sequence.SetTime(Time(621.0));
  EXPECT_EQ(sequence.Next(), 2u);
}

TEST(LightBuoySequence, PauseAndRewindAreHandledInSimulationTime)
{
  LightBuoySequence sequence;
  EXPECT_FALSE(sequence.Next());
  sequence.SetTime(Time(200.0));
  EXPECT_EQ(sequence.Next(), 0u);
  for (int frame = 0; frame < 100; ++frame)
  {
    sequence.SetTime(Time(200.0));
    EXPECT_FALSE(sequence.Next());
  }
  sequence.SetTime(Time(201.0));
  EXPECT_EQ(sequence.Next(), 1u);
  sequence.SetTime(Time(0.0));
  EXPECT_EQ(sequence.Next(), 0u);
  EXPECT_FALSE(sequence.Next());
  sequence.SetTime(Time(1.0));
  EXPECT_EQ(sequence.Next(), 1u);
}
