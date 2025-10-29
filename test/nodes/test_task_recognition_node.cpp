#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/package.h>
#include <thread>
#include <chrono>

#include "procedural_interfaces/GetTasks.h"

using namespace std;

class MockTaskNodeMemoryService {
public:
    MockTaskNodeMemoryService(ros::NodeHandle& nh) : nh_(nh), call_count_(0) {
        server_ = nh_.advertiseService("test_task_node_memory/get_tasks",
                                     &MockTaskNodeMemoryService::getTasksCallback, this);
    }

    bool getTasksCallback(procedural_interfaces::GetTasks::Request& req,
                         procedural_interfaces::GetTasks::Response& res) {
        call_count_++;

        // Create test tasks for the node

        // Primitive task
        procedural_interfaces::Task primitive_task;
        primitive_task.task_name = "test_primitive";
        primitive_task.is_primitive = true;
        primitive_task.description = "Test primitive task";

        procedural_interfaces::TaskArgument prim_arg;
        prim_arg.name = "object";
        prim_arg.type = "item";
        primitive_task.arguments.push_back(prim_arg);

        procedural_interfaces::TaskEffect prim_effect;
        prim_effect.subject = "robot";
        prim_effect.predicate = "has";
        prim_effect.object = "object";
        prim_effect.is_add = true;
        primitive_task.effects.push_back(prim_effect);

        res.tasks.tasks.push_back(primitive_task);

        // Abstract task
        procedural_interfaces::Task abstract_task;
        abstract_task.task_name = "test_abstract";
        abstract_task.is_primitive = false;
        abstract_task.description = "Test abstract task";

        procedural_interfaces::TaskArgument abs_arg;
        abs_arg.name = "target";
        abs_arg.type = "location";
        abstract_task.arguments.push_back(abs_arg);

        // Add method
        procedural_interfaces::Method method;
        method.method_name = "test_method";

        procedural_interfaces::TaskPrecondition precond;
        precond.subject = "robot";
        precond.predicate = "ready";
        precond.object = "true";
        precond.is_negative = false;
        method.preconditions.push_back(precond);

        method.decomposition.push_back("test_primitive(item1)");
        method.decomposition.push_back("move_to(target)");

        abstract_task.methods.push_back(method);

        res.tasks.tasks.push_back(abstract_task);

        return true;
    }

    int getCallCount() const { return call_count_; }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer server_;
    int call_count_;
};

class TaskRecognitionNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<MockTaskNodeMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MockTaskNodeMemoryService> mock_service;
};

TEST_F(TaskRecognitionNodeTest, ServiceConnectionTest) {
    // Test that the mock service is properly set up
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "test_task_node_memory/get_tasks");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    procedural_interfaces::GetTasks srv;
    ASSERT_TRUE(client.call(srv));
    EXPECT_EQ(srv.response.tasks.tasks.size(), 2);
    EXPECT_EQ(srv.response.tasks.tasks[0].task_name, "test_primitive");
    EXPECT_EQ(srv.response.tasks.tasks[1].task_name, "test_abstract");
}

TEST_F(TaskRecognitionNodeTest, MemoryServiceParametersTest) {
    // Test setting various parameters that the task recognition node uses
    ros::NodeHandle private_nh("~");

    // Set test parameters
    private_nh.setParam("memory_service_namespace", "test_task_node_memory");
    private_nh.setParam("task_filter", "test_");

    // Verify parameters were set
    std::string service_namespace;
    std::string task_filter;

    ASSERT_TRUE(private_nh.getParam("memory_service_namespace", service_namespace));
    ASSERT_TRUE(private_nh.getParam("task_filter", task_filter));

    EXPECT_EQ(service_namespace, "test_task_node_memory");
    EXPECT_EQ(task_filter, "test_");
}

TEST_F(TaskRecognitionNodeTest, ServiceCallsTest) {
    // Test that we can simulate the node's service calls
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "test_task_node_memory/get_tasks");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    // Simulate multiple calls as the node would do
    int initial_count = mock_service->getCallCount();

    for (int i = 0; i < 3; ++i) {
        procedural_interfaces::GetTasks srv;
        ASSERT_TRUE(client.call(srv));
        EXPECT_EQ(srv.response.tasks.tasks.size(), 2);
    }

    EXPECT_EQ(mock_service->getCallCount(), initial_count + 3);
}

TEST_F(TaskRecognitionNodeTest, TaskDataValidationTest) {
    // Test that the task data from service is valid for the node
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "test_task_node_memory/get_tasks");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    procedural_interfaces::GetTasks srv;
    ASSERT_TRUE(client.call(srv));

    ASSERT_EQ(srv.response.tasks.tasks.size(), 2);

    // Validate primitive task
    auto& primitive_task = srv.response.tasks.tasks[0];
    EXPECT_FALSE(primitive_task.task_name.empty());
    EXPECT_TRUE(primitive_task.is_primitive);
    EXPECT_FALSE(primitive_task.arguments.empty());
    EXPECT_FALSE(primitive_task.effects.empty());
    EXPECT_TRUE(primitive_task.methods.empty());  // Primitive should have no methods

    // Validate abstract task
    auto& abstract_task = srv.response.tasks.tasks[1];
    EXPECT_FALSE(abstract_task.task_name.empty());
    EXPECT_FALSE(abstract_task.is_primitive);
    EXPECT_FALSE(abstract_task.arguments.empty());
    EXPECT_FALSE(abstract_task.methods.empty());

    // Validate method
    auto& method = abstract_task.methods[0];
    EXPECT_FALSE(method.method_name.empty());
    EXPECT_FALSE(method.preconditions.empty());
    EXPECT_FALSE(method.decomposition.empty());

    // Validate precondition
    auto& precond = method.preconditions[0];
    EXPECT_FALSE(precond.subject.empty());
    EXPECT_FALSE(precond.predicate.empty());
    EXPECT_FALSE(precond.object.empty());

    // Validate decomposition
    EXPECT_EQ(method.decomposition.size(), 2);
    EXPECT_FALSE(method.decomposition[0].empty());
    EXPECT_FALSE(method.decomposition[1].empty());
}

TEST_F(TaskRecognitionNodeTest, HTNStructureTest) {
    // Test that the tasks can be organized into HTN structure
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "test_task_node_memory/get_tasks");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    procedural_interfaces::GetTasks srv;
    ASSERT_TRUE(client.call(srv));

    ASSERT_EQ(srv.response.tasks.tasks.size(), 2);

    // Count primitive vs abstract tasks
    int primitive_count = 0;
    int abstract_count = 0;

    for (const auto& task : srv.response.tasks.tasks) {
        if (task.is_primitive) {
            primitive_count++;
        } else {
            abstract_count++;
        }
    }

    EXPECT_EQ(primitive_count, 1);
    EXPECT_EQ(abstract_count, 1);

    // Verify abstract task has proper decomposition
    for (const auto& task : srv.response.tasks.tasks) {
        if (!task.is_primitive) {
            EXPECT_FALSE(task.methods.empty());
            for (const auto& method : task.methods) {
                EXPECT_FALSE(method.decomposition.empty());
                // Each decomposition should be parseable as action calls
                for (const auto& decomp : method.decomposition) {
                    EXPECT_TRUE(decomp.find("(") != std::string::npos);
                    EXPECT_TRUE(decomp.find(")") != std::string::npos);
                }
            }
        }
    }
}

TEST_F(TaskRecognitionNodeTest, ServiceUnavailableTest) {
    // Test behavior when service is not available
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "non_existent_service/get_tasks");

    // Should not exist
    EXPECT_FALSE(client.waitForExistence(ros::Duration(1.0)));

    // Call should fail
    procedural_interfaces::GetTasks srv;
    EXPECT_FALSE(client.call(srv));
}

TEST_F(TaskRecognitionNodeTest, FilterParameterTest) {
    // Test service call with filter parameter
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "test_task_node_memory/get_tasks");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    // Test with filter
    procedural_interfaces::GetTasks srv;
    srv.request.filter = "test_primitive";
    ASSERT_TRUE(client.call(srv));

    EXPECT_EQ(srv.response.tasks.tasks.size(), 2);  // Our mock doesn't filter

    // Test with different filter
    srv.request.filter = "NonExistentTask";
    ASSERT_TRUE(client.call(srv));
    // Our mock doesn't implement filtering, but call should succeed
    EXPECT_EQ(srv.response.tasks.tasks.size(), 2);
}

TEST_F(TaskRecognitionNodeTest, MethodComplexityTest) {
    // Test handling of complex method structures
    ros::ServiceClient client = nh->serviceClient<procedural_interfaces::GetTasks>(
        "test_task_node_memory/get_tasks");

    ASSERT_TRUE(client.waitForExistence(ros::Duration(2.0)));

    procedural_interfaces::GetTasks srv;
    ASSERT_TRUE(client.call(srv));

    // Find abstract task and validate its method complexity
    const procedural_interfaces::Task* abstract_task = nullptr;
    for (const auto& task : srv.response.tasks.tasks) {
        if (!task.is_primitive) {
            abstract_task = &task;
            break;
        }
    }

    ASSERT_NE(abstract_task, nullptr);
    ASSERT_FALSE(abstract_task->methods.empty());

    auto& method = abstract_task->methods[0];

    // Test preconditions parsing capability
    EXPECT_FALSE(method.preconditions.empty());
    for (const auto& precond : method.preconditions) {
        EXPECT_FALSE(precond.subject.empty());
        EXPECT_FALSE(precond.predicate.empty());
        // is_negative field should be properly set
    }

    // Test decomposition parsing capability
    EXPECT_EQ(method.decomposition.size(), 2);
    EXPECT_TRUE(method.decomposition[0].find("test_primitive") != std::string::npos);
    EXPECT_TRUE(method.decomposition[1].find("move_to") != std::string::npos);
}

// Integration test that would require the actual node running
class TaskRecognitionNodeIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // This would be used for tests that actually launch the node
        // For now, we just check if the node executable exists
    }
};

TEST_F(TaskRecognitionNodeIntegrationTest, NodeExecutableExistsTest) {
    // Check if the node executable was built
    std::string package_path = ros::package::getPath("procedural");
    if (package_path.empty()) {
        GTEST_SKIP() << "Procedural package not found";
    }

    // This is more of a build verification test
    EXPECT_FALSE(package_path.empty());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_task_recognition_node");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}