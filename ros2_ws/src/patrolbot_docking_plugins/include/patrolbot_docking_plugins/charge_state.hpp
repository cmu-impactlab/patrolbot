#ifndef PATROLBOT_DOCKING_PLUGINS__CHARGE_STATE_HPP_
#define PATROLBOT_DOCKING_PLUGINS__CHARGE_STATE_HPP_

#include <cstdint>
#include <string>

namespace patrolbot_docking_plugins
{

inline bool isDockedState(const bool valid, const std::string & state)
{
  return valid && state == "DOCKED_CONFIRMED";
}

inline bool isChargingState(
  const bool charge_valid, const int16_t charge_state,
  const bool dock_valid, const std::string & dock_state)
{
  // Raw state 3 can re-latch after physical clearance. Charging is true only
  // when the SBC's fresh physical dock observer corroborates it.
  return charge_valid && charge_state > 0 &&
         isDockedState(dock_valid, dock_state);
}

inline bool isReleasedState(const bool valid, const int16_t state)
{
  return valid && state == 0;
}

}  // namespace patrolbot_docking_plugins

#endif  // PATROLBOT_DOCKING_PLUGINS__CHARGE_STATE_HPP_
