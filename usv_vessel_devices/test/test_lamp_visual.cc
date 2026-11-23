#include <gtest/gtest.h>
#include "usv_vessel_devices/VisualName.hh"

TEST(LampVisual, MatchesOriginalAndConvertedLensButNeverHousing)
{
  using usv_vessel_devices::MatchesVisualName;
  EXPECT_TRUE(MatchesVisualName("lamp", "lamp"));
  EXPECT_TRUE(MatchesVisualName("usv_1/light_left_front_link_fixed_joint_lump__lamp_visual", "lamp"));
  EXPECT_TRUE(MatchesVisualName("usv_1/light_left_front_link_fixed_joint_lump__lamp_visual_1", "lamp"));
  EXPECT_FALSE(MatchesVisualName("usv_1/light_left_front_link_fixed_joint_lump__lamp_visual_bad", "lamp"));
  EXPECT_FALSE(MatchesVisualName("housing", "lamp"));
  EXPECT_FALSE(MatchesVisualName("usv_1/light_left_front_link_fixed_joint_lump__housing_visual", "lamp"));
  EXPECT_FALSE(MatchesVisualName("lamp", ""));
}
