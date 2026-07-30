#include <gtest/gtest.h>

#include "patrolbot_docking_plugins/charge_state.hpp"

TEST(ChargeState, RequiresValidity)
{
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(
    false, 2, true, "DOCKED_CONFIRMED"));
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(
    true, 2, false, "DOCKED_CONFIRMED"));
  EXPECT_FALSE(patrolbot_docking_plugins::isReleasedState(false, 0));
}

TEST(ChargeState, DistinguishesReleasedAndChargingStates)
{
  EXPECT_TRUE(patrolbot_docking_plugins::isReleasedState(true, 0));
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(
    true, 0, true, "DOCKED_CONFIRMED"));
  for (const int16_t charging_state : {1, 2, 3}) {
    EXPECT_TRUE(
      patrolbot_docking_plugins::isChargingState(
        true, charging_state, true, "DOCKED_CONFIRMED"));
    EXPECT_FALSE(
      patrolbot_docking_plugins::isReleasedState(true, charging_state));
  }
  EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(
    true, -1, true, "DOCKED_CONFIRMED"));
  EXPECT_FALSE(patrolbot_docking_plugins::isReleasedState(true, -1));
}

TEST(ChargeState, StaleRawChargeDoesNotMeanDockedOrCharging)
{
  for (const int16_t stale_state : {2, 3}) {
    EXPECT_FALSE(patrolbot_docking_plugins::isChargingState(
      true, stale_state, true, "CLEAR_CONFIRMED"));
  }
  EXPECT_FALSE(
    patrolbot_docking_plugins::isDockedState(true, "CLEAR_CONFIRMED"));
  EXPECT_TRUE(
    patrolbot_docking_plugins::isDockedState(true, "DOCKED_CONFIRMED"));
}
