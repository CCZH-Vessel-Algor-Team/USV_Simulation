#ifndef USV_VESSEL_DEVICES_VISUAL_NAME_HH_
#define USV_VESSEL_DEVICES_VISUAL_NAME_HH_

#include <string>
#include <algorithm>
#include <cctype>

namespace usv_vessel_devices
{
inline bool MatchesVisualName(const std::string &actual, const std::string &requested)
{
  if (requested.empty()) return false;
  if (actual == requested) return true;
  // sdformat renames a named URDF visual, even for a preserved fixed link.
  const std::string suffix = "__" + requested + "_visual";
  const auto start = actual.rfind(suffix);
  if (start == std::string::npos) return false;
  const auto tail = actual.substr(start + suffix.size());
  if (tail.empty()) return true;
  // Additional visuals receive numeric suffixes, e.g. __lamp_visual_1.
  return tail.size() > 1 && tail.front() == '_' &&
    std::all_of(tail.begin() + 1, tail.end(),
      [](unsigned char ch) {return std::isdigit(ch) != 0;});
}
}  // namespace usv_vessel_devices
#endif
