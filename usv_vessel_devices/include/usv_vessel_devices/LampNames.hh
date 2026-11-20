#ifndef USV_VESSEL_DEVICES_LAMP_NAMES_HH_
#define USV_VESSEL_DEVICES_LAMP_NAMES_HH_

#include <algorithm>
#include <array>
#include <cctype>
#include <set>
#include <string>
#include <vector>

namespace usv_vessel_devices
{
inline bool ValidLampNames(const std::vector<std::string> &names)
{
  const std::array<std::string, 5> legacy{
    "left_front", "left_rear", "right_front", "right_rear", "mast"};
  if (names.size() < legacy.size() || !std::equal(legacy.begin(), legacy.end(), names.begin()))
    return false;
  std::set<std::string> unique;
  for (const auto &name : names)
  {
    if (name.empty() || (!std::isalpha(static_cast<unsigned char>(name.front())) && name.front() != '_') ||
      !std::all_of(name.begin(), name.end(), [](unsigned char ch)
        {return std::isalnum(ch) || ch == '_';}) || !unique.insert(name).second)
      return false;
  }
  return true;
}
}  // namespace usv_vessel_devices
#endif
