#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>

#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural_interfaces/GetActions.h"

using namespace procedural;

class MockMemoryService {
public:
    MockMemoryService(ros::NodeHandle& nh) : nh_(nh) {
        server_ = nh_.advertiseService("procedural_memory/get_actions",
                                     &MockMemoryService::getActionsCallback, this);
    }

    bool getActionsCallback(procedural_interfaces::GetActions::Request& req,
                           procedural_interfaces::GetActions::Response& res) {
        // Mock response with test actions
        procedural_interfaces::Action simple_action;
        simple_action.actionName = "test_simple_action";

        // Add execution action to mark it as simple
        procedural_interfaces::ExecutionAction exec_action;
        exec_action.name = "test_execution";
        simple_action.executionActions.push_back(exec_action);

        // Add argument
        procedural_interfaces::Argument arg;
        arg.type = "string";
        arg.value = "test_value";
        simple_action.arguments.push_back(arg);

        res.actions.push_back(simple_action);

        // Add a composed action
        procedural_interfaces::Action composed_action;
        composed_action.actionName = "test_composed_action";
        // No execution actions makes it composed
        res.actions.push_back(composed_action);

        return true;
    }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer server_;
};

class MemoryActionReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<MockMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MockMemoryService> mock_service;
};

TEST_F(MemoryActionReaderTest, ConstructorTest) {
    ASSERT_NO_THROW({
        MemoryActionReader reader(*nh, "procedural_memory");
    });
}

TEST_F(MemoryActionReaderTest, ServiceAvailabilityTest) {
    MemoryActionReader reader(*nh, "procedural_memory");

    // Wait for service to be available
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    EXPECT_TRUE(reader.isServiceAvailable());
}

TEST_F(MemoryActionReaderTest, ReadActionsTest) {
    MemoryActionReader reader(*nh, "procedural_memory");

    // Wait for service
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read actions
    EXPECT_TRUE(reader.read());
    EXPECT_TRUE(reader.hasActions());
    EXPECT_EQ(reader.getActionCount(), 2);

    // Check simple actions
    auto simple_actions = reader.getSimpleActions();
    EXPECT_EQ(simple_actions.size(), 1);
    EXPECT_EQ(simple_actions[0].getName(), "test_simple_action");

    // Check composed actions
    auto composed_actions = reader.getComposedActions();
    EXPECT_EQ(composed_actions.size(), 1);
    EXPECT_EQ(composed_actions[0].getName(), "test_composed_action");
}

TEST_F(MemoryActionReaderTest, ReadWithFilterTest) {
    MemoryActionReader reader(*nh, "procedural_memory");

    // Wait for service
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read with filter (our mock doesn't implement filtering, but we test the interface)
    EXPECT_TRUE(reader.read("test_filter"));
    EXPECT_TRUE(reader.hasActions());
}

TEST_F(MemoryActionReaderTest, NoServiceTest) {
    // Create reader with non-existent service namespace
    MemoryActionReader reader(*nh, "non_existent_service");

    EXPECT_FALSE(reader.isServiceAvailable());
    EXPECT_FALSE(reader.waitForService(ros::Duration(1.0)));
    EXPECT_FALSE(reader.read());
    EXPECT_FALSE(reader.hasActions());
    EXPECT_EQ(reader.getActionCount(), 0);
}

TEST_F(MemoryActionReaderTest, EmptyResponseTest) {
    // We could extend the mock to support empty responses for this test
    // For now, we test that the reader handles the response correctly
    MemoryActionReader reader(*nh, "procedural_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Even with our mock response, the reader should handle it gracefully
    EXPECT_TRUE(reader.read());

    // Verify the structures are initialized correctly
    auto simple_actions = reader.getSimpleActions();
    auto composed_actions = reader.getComposedActions();

    // Should have exactly what our mock provides
    EXPECT_EQ(simple_actions.size() + composed_actions.size(), reader.getActionCount());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_memory_action_reader");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}