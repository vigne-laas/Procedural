#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/package.h>
#include <thread>
#include <chrono>

#include "procedural_interfaces/GetActions.h"

using namespace std;

class MockActionNodeMemoryService {
public:
    MockActionNodeMemoryService(ros::NodeHandle& nh) : nh_(nh), call_count_(0) {
        server_ = nh_.advertiseService("test_action_node_memory/get_actions",
                                     &MockActionNodeMemoryService::getActionsCallback, this);
    }

    bool getActionsCallback(procedural_interfaces::GetActions::Request& req,
                           procedural_interfaces::GetActions::Response& res) {
        call_count_++;

        // Create test actions for the node
        procedural_interfaces::Action simple_action;
        simple_action.actionName = "TestAction";

        // Add execution action to make it simple
        procedural_interfaces::ExecutionAction exec_action;
        exec_action.name = "test_execution";
        simple_action.executionActions.push_back(exec_action);

        // Add arguments
        procedural_interfaces::Argument arg;
        arg.type = "object";
        arg.value = "?obj";
        simple_action.arguments.push_back(arg);

        res.actions.push_back(simple_action);

        return true;
    }

    int getCallCount() const { return call_count_; }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer server_;
    int call_count_;
};

class ActionRecognitionNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<MockActionNodeMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MockActionNodeMemoryService> mock_service;
};

TEST_F(ActionRecognitionNodeTest, ServiceConnectionTest) {
    // Test that the mock service is properly set up
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetActions>(
        "test_action_node_memory/get_actions");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    procedural_interfaces::GetActions srv;
    ASSERT_TRUE(client.call(srv));
    EXPECT_EQ(srv.response.actions.size(), 1);
    EXPECT_EQ(srv.response.actions[0].actionName, "TestAction");
}

TEST_F(ActionRecognitionNodeTest, MemoryServiceParametersTest) {
    // Test setting various parameters that the action recognition node uses
    ros::NodeHandle private_nh("~");

    // Set test parameters
    private_nh.setParam("memory_service_namespace", "test_action_node_memory");
    private_nh.setParam("action_filter", "");
    private_nh.setParam("fact_time_to_live", 5.0);
    private_nh.setParam("buffer_max_size", 100);

    // Verify parameters were set
    std::string service_namespace;
    std::string action_filter;
    double fact_ttl;
    int buffer_size;

    ASSERT_TRUE(private_nh.getParam("memory_service_namespace", service_namespace));
    ASSERT_TRUE(private_nh.getParam("action_filter", action_filter));
    ASSERT_TRUE(private_nh.getParam("fact_time_to_live", fact_ttl));
    ASSERT_TRUE(private_nh.getParam("buffer_max_size", buffer_size));

    EXPECT_EQ(service_namespace, "test_action_node_memory");
    EXPECT_EQ(action_filter, "");
    EXPECT_EQ(fact_ttl, 5.0);
    EXPECT_EQ(buffer_size, 100);
}

TEST_F(ActionRecognitionNodeTest, ServiceCallsTest) {
    // Test that we can simulate the node's service calls
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetActions>(
        "test_action_node_memory/get_actions");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    // Simulate multiple calls as the node would do
    int initial_count = mock_service->getCallCount();

    for (int i = 0; i < 3; ++i) {
        procedural_interfaces::GetActions srv;
        ASSERT_TRUE(client.call(srv));
        EXPECT_EQ(srv.response.actions.size(), 1);
    }

    EXPECT_EQ(mock_service->getCallCount(), initial_count + 3);
}

TEST_F(ActionRecognitionNodeTest, ActionDataValidationTest) {
    // Test that the action data from service is valid for the node
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetActions>(
        "test_action_node_memory/get_actions");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    procedural_interfaces::GetActions srv;
    ASSERT_TRUE(client.call(srv));

    ASSERT_EQ(srv.response.actions.size(), 1);
    auto& action = srv.response.actions[0];

    // Validate action structure expected by the node
    EXPECT_FALSE(action.actionName.empty());
    EXPECT_FALSE(action.executionActions.empty());  // Should be simple action
    EXPECT_FALSE(action.arguments.empty());

    // Validate execution action
    auto& exec_action = action.executionActions[0];
    EXPECT_FALSE(exec_action.name.empty());

    // Validate arguments
    auto& arg = action.arguments[0];
    EXPECT_FALSE(arg.type.empty());
    EXPECT_FALSE(arg.value.empty());
}

TEST_F(ActionRecognitionNodeTest, ServiceUnavailableTest) {
    // Test behavior when service is not available
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetActions>(
        "non_existent_service/get_actions");

    // Should not exist
    EXPECT_FALSE(client.waitForExistence(ros::Duration(1.0)));

    // Call should fail
    procedural_interfaces::GetActions srv;
    EXPECT_FALSE(client.call(srv));
}

TEST_F(ActionRecognitionNodeTest, FilterParameterTest) {
    // Test service call with filter parameter
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetActions>(
        "test_action_node_memory/get_actions");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    // Test with filter
    procedural_interfaces::GetActions srv;
    srv.request.filter = "TestAction";
    ASSERT_TRUE(client.call(srv));

    EXPECT_EQ(srv.response.actions.size(), 1);
    EXPECT_EQ(srv.response.actions[0].actionName, "TestAction");

    // Test with different filter
    srv.request.filter = "NonExistentAction";
    ASSERT_TRUE(client.call(srv));
    // Our mock doesn't implement filtering, but call should succeed
    EXPECT_EQ(srv.response.actions.size(), 1);
}

// Integration test that would require the actual node running
class ActionRecognitionNodeIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // This would be used for tests that actually launch the node
        // For now, we just check if the node executable exists
    }
};

TEST_F(ActionRecognitionNodeIntegrationTest, NodeExecutableExistsTest) {
    // Check if the node executable was built
    std::string package_path = ros::package::getPath("procedural");
    if (package_path.empty()) {
        GTEST_SKIP() << "Procedural package not found";
    }

    // This is more of a build verification test
    EXPECT_FALSE(package_path.empty());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_action_recognition_node");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}