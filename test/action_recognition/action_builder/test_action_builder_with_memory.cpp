#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>

#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural/action_recognition/builder/ActionBuilder.h"
#include "procedural_interfaces/GetActions.h"

using namespace procedural;

class MockActionMemoryService {
public:
    MockActionMemoryService(ros::NodeHandle& nh) : nh_(nh) {
        server_ = nh_.advertiseService("test_memory/get_actions",
                                     &MockActionMemoryService::getActionsCallback, this);
    }

    bool getActionsCallback(procedural_interfaces::GetActions::Request& req,
                           procedural_interfaces::GetActions::Response& res) {
        // Create comprehensive test actions for ActionBuilder testing

        // Simple action 1: Release
        procedural_interfaces::Action release_action;
        release_action.actionName = "Release";

        // Add execution action
        procedural_interfaces::ExecutionAction exec_release;
        exec_release.name = "release_gripper";
        release_action.executionActions.push_back(exec_release);

        // Add arguments
        procedural_interfaces::Argument rel_arg1;
        rel_arg1.type = "object";
        rel_arg1.value = "?obj";
        release_action.arguments.push_back(rel_arg1);

        res.actions.push_back(release_action);

        // Simple action 2: Grasp
        procedural_interfaces::Action grasp_action;
        grasp_action.actionName = "Grasp";

        procedural_interfaces::ExecutionAction exec_grasp;
        exec_grasp.name = "close_gripper";
        grasp_action.executionActions.push_back(exec_grasp);

        procedural_interfaces::Argument grasp_arg1;
        grasp_arg1.type = "object";
        grasp_arg1.value = "?target";
        grasp_action.arguments.push_back(grasp_arg1);

        res.actions.push_back(grasp_action);

        // Composed action: PickUp (uses Release and Grasp)
        procedural_interfaces::Action pickup_action;
        pickup_action.actionName = "PickUp";
        // No execution actions makes it composed

        procedural_interfaces::Argument pickup_arg1;
        pickup_arg1.type = "object";
        pickup_arg1.value = "?item";
        pickup_action.arguments.push_back(pickup_arg1);

        res.actions.push_back(pickup_action);

        return true;
    }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer server_;
};

class ActionBuilderWithMemoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<MockActionMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MockActionMemoryService> mock_service;
};

TEST_F(ActionBuilderWithMemoryTest, BuildActionsFromMemoryTest) {
    // Create memory reader
    MemoryActionReader reader(*nh, "test_memory");

    // Wait for service and read actions
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    // Verify actions were read
    EXPECT_TRUE(reader.hasActions());
    EXPECT_EQ(reader.getActionCount(), 3);

    auto simple_actions = reader.getSimpleActions();
    auto composed_actions = reader.getComposedActions();

    EXPECT_EQ(simple_actions.size(), 2);   // Release, Grasp
    EXPECT_EQ(composed_actions.size(), 1); // PickUp

    // Test ActionBuilder with memory-loaded actions
    ASSERT_NO_THROW({
        ActionBuilder builder(simple_actions, composed_actions, "/tmp/test_debug");
        auto built_actions = builder.getActions();

        // Should have built all actions successfully
        EXPECT_EQ(built_actions.size(), 3);

        // Verify action names
        std::set<std::string> action_names;
        for (const auto& action : built_actions) {
            action_names.insert(action->getName());
        }

        EXPECT_TRUE(action_names.count("Release") > 0);
        EXPECT_TRUE(action_names.count("Grasp") > 0);
        EXPECT_TRUE(action_names.count("PickUp") > 0);
    });
}

TEST_F(ActionBuilderWithMemoryTest, ActionBuilderConsistencyTest) {
    // Test that ActionBuilder produces consistent results with memory vs direct data

    MemoryActionReader reader(*nh, "test_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto simple_actions = reader.getSimpleActions();
    auto composed_actions = reader.getComposedActions();

    // Build actions first time
    ActionBuilder builder1(simple_actions, composed_actions, "/tmp/test_debug1");
    auto actions1 = builder1.getActions();

    // Build actions second time with same data
    ActionBuilder builder2(simple_actions, composed_actions, "/tmp/test_debug2");
    auto actions2 = builder2.getActions();

    // Should produce same number of actions
    EXPECT_EQ(actions1.size(), actions2.size());

    // Verify all action names match
    std::set<std::string> names1, names2;
    for (const auto& action : actions1) names1.insert(action->getName());
    for (const auto& action : actions2) names2.insert(action->getName());

    EXPECT_EQ(names1, names2);
}

TEST_F(ActionBuilderWithMemoryTest, FilteredActionsTest) {
    // Test building actions with filtered input
    MemoryActionReader reader(*nh, "test_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read with potential filter (our mock doesn't filter, but test the interface)
    ASSERT_TRUE(reader.read("Grasp"));

    auto simple_actions = reader.getSimpleActions();
    auto composed_actions = reader.getComposedActions();

    // Should still get all actions from our mock
    EXPECT_TRUE(simple_actions.size() + composed_actions.size() > 0);

    // ActionBuilder should handle any subset of actions
    ASSERT_NO_THROW({
        ActionBuilder builder(simple_actions, composed_actions);
        auto built_actions = builder.getActions();
        EXPECT_GT(built_actions.size(), 0);
    });
}

TEST_F(ActionBuilderWithMemoryTest, EmptyActionsTest) {
    // Test ActionBuilder behavior with empty action lists
    std::vector<ParsedSimpleAction_t> empty_simple;
    std::vector<ParsedComposedAction_t> empty_composed;

    // ActionBuilder should handle empty inputs gracefully
    ASSERT_NO_THROW({
        ActionBuilder builder(empty_simple, empty_composed);
        auto built_actions = builder.getActions();
        EXPECT_EQ(built_actions.size(), 0);
    });
}

TEST_F(ActionBuilderWithMemoryTest, ActionStructureValidationTest) {
    MemoryActionReader reader(*nh, "test_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto simple_actions = reader.getSimpleActions();

    // Verify the converted action structures have correct properties
    for (const auto& action : simple_actions) {
        EXPECT_FALSE(action.getName().empty());
        // Verify the action has proper type/subtype parsing
        EXPECT_FALSE(action.type.empty());
    }

    // Test with ActionBuilder
    ActionBuilder builder(simple_actions, reader.getComposedActions());
    auto built_actions = builder.getActions();

    for (const auto& action : built_actions) {
        EXPECT_FALSE(action->getName().empty());
        // Verify action was built with proper internal structure
        EXPECT_NE(action, nullptr);
    }
}

TEST_F(ActionBuilderWithMemoryTest, MemoryReaderReuseTest) {
    // Test that MemoryActionReader can be reused multiple times
    MemoryActionReader reader(*nh, "test_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read multiple times
    ASSERT_TRUE(reader.read());
    size_t first_count = reader.getActionCount();

    ASSERT_TRUE(reader.read());
    size_t second_count = reader.getActionCount();

    // Should get consistent results
    EXPECT_EQ(first_count, second_count);

    // ActionBuilder should work with any read
    auto simple_actions = reader.getSimpleActions();
    auto composed_actions = reader.getComposedActions();

    ActionBuilder builder(simple_actions, composed_actions);
    auto built_actions = builder.getActions();
    EXPECT_GT(built_actions.size(), 0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_action_builder_with_memory");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}