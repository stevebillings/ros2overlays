#include <gtest/gtest.h>
#include "../../../src/fsm_gof/state/state_near.h"

TEST(StateNearTest, Name)
{
  StateNear state_near = StateNear();
  EXPECT_STREQ("obstacle near", state_near.name());
}

// TODO More tests here