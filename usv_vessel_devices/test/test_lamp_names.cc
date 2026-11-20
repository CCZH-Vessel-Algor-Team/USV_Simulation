#include <gtest/gtest.h>
#include "usv_vessel_devices/LampNames.hh"

using usv_vessel_devices::ValidLampNames;

TEST(LampNames, LegacyPrefixAndOptionalUniqueExtras)
{
  std::vector<std::string> names{"left_front", "left_rear", "right_front", "right_rear", "mast"};
  EXPECT_TRUE(ValidLampNames(names));
  names.push_back("aft_lower");
  EXPECT_TRUE(ValidLampNames(names));
  names.push_back("aft_lower");
  EXPECT_FALSE(ValidLampNames(names));
  names.back() = "mast";
  EXPECT_FALSE(ValidLampNames(names));
  for (const auto &invalid : {"", "2lamp", "aft/lower", "aft-lower", "aft lower"})
  {
    names.back() = invalid;
    EXPECT_FALSE(ValidLampNames(names));
  }
  names.pop_back();
  std::swap(names[0], names[1]);
  EXPECT_FALSE(ValidLampNames(names));
  EXPECT_FALSE(ValidLampNames({}));
  EXPECT_FALSE(ValidLampNames({"left_front"}));
}
