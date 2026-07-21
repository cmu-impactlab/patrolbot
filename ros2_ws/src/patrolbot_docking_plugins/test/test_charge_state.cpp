#include <gtest/gtest.h>

#include "patrolbot_docking_plugins/charge_state.hpp"

TEST(ChargeState, RequiresValidity)
{
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(false, 2));
  EXPECT_FALSE(patrolbot_docking_plugins::isReleasedState(false, 0));
}

TEST(ChargeState, DistinguishesReleasedAndChargingStates)
{
  EXPECT_TRUE(patrolbot_docking_plugins::isReleasedState(true, 0));
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(true, 0));
  for (const int16_t charging_state : {1, 2, 3}) {
    EXPECT_TRUE(
      patrolbot_docking_plugins::isChargingState(true, charging_state));
    EXPECT_FALSE(
      patrolbot_docking_plugins::isReleasedState(true, charging_state));
  }
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(true, -1));
  EXPECT_FALSE(patrolbot_docking_plugins::isReleasedState(true, -1));
}
