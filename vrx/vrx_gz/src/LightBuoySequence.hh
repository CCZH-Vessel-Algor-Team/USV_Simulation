#ifndef VRX_LIGHT_BUOY_SEQUENCE_HH_
#define VRX_LIGHT_BUOY_SEQUENCE_HH_

#include <chrono>
#include <cstddef>
#include <optional>

namespace vrx
{
/// \brief Schedule the five-color cycle from the buoy's own simulation time.
/// Late rendering advances one color and starts a new one-second interval,
/// rather than replaying updates missed before the buoy or scene existed.
class LightBuoySequence
{
  public: using Time = std::chrono::duration<double>;

  /// \brief Observe simulation time; a rewind restarts the color sequence.
  public: void SetTime(Time _time)
  {
    if (!this->currentTime || _time < *this->currentTime)
    {
      this->state = 0u;
      this->nextUpdateTime = _time;
    }
    this->currentTime = _time;
  }

  /// \brief Return a color index only when a new color should be displayed.
  /// Call only after the visuals are ready. An unchanged simulation time
  /// (including a paused simulation) cannot advance the sequence.
  public: std::optional<std::size_t> Next()
  {
    if (!this->currentTime || *this->currentTime < this->nextUpdateTime)
      return std::nullopt;

    const auto result = this->state;
    this->state = (this->state + 1u) % 5u;
    this->nextUpdateTime = *this->currentTime + Time(1.0);
    return result;
  }

  private: std::optional<Time> currentTime;
  private: Time nextUpdateTime{0.0};
  private: std::size_t state{0u};
};
}

#endif
