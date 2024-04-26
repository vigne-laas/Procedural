#include <gtest/gtest.h>
#include <ros/ros.h>
#include "procedural/action_recognition/core/internal_structures/state_machine/State.h"

TEST(StateTest, testStateCreation) {
    procedural::State state("test_state", 1);
    EXPECT_EQ(state.getId(), 1);
    EXPECT_EQ(state.getFullName(), "test_state_1");
}

TEST(StateTest, testStateTransition) {
    procedural::State state1("state1", 1);
    procedural::State state2("state2", 2);
    procedural::ActionTransition_t transition;
    state1.addTransition(transition, &state2);
    EXPECT_EQ(state1.getNextState().size(), 1);
    EXPECT_EQ(state1.getNextState()[0].second->getId(), state2.getId());
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "state_test");
    return RUN_ALL_TESTS();
}