#ifndef PATROLBOT_DOCKING_PLUGINS__CHARGE_STATE_HPP_
#define PATROLBOT_DOCKING_PLUGINS__CHARGE_STATE_HPP_

#include <cstdint>

namespace patrolbot_docking_plugins
{

inline bool isChargingState(const bool valid, const int16_t state)
{
  return valid && state > 0;
}

inline bool isReleasedState(const bool valid, const int16_t state)
{
  return valid && state == 0;
}

}  // namespace patrolbot_docking_plugins

#endif  // PATROLBOT_DOCKING_PLUGINS__CHARGE_STATE_HPP_
