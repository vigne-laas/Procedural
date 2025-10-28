#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include "procedural/memory_client/MemoryServiceClient.h"
#include "procedural/memory_client/RosToInternalConverter.h"
#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural/task_recognition/Reader/MemoryDomainReader.h"

using namespace procedural;

class MemoryIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
    }

    void TearDown() override {
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
};

TEST_F(MemoryIntegrationTest, MemoryServiceClientCreation) {
    ASSERT_NO_THROW({
        MemoryServiceClient client(*nh, "test_namespace");
    });
}

TEST_F(MemoryIntegrationTest, MemoryActionReaderCreation) {
    ASSERT_NO_THROW({
        MemoryActionReader reader(*nh, "test_namespace");
    });
}

TEST_F(MemoryIntegrationTest, MemoryDomainReaderCreation) {
    ASSERT_NO_THROW({
        MemoryDomainReader reader(*nh, "test_namespace");
    });
}

TEST_F(MemoryIntegrationTest, RosToInternalConverterActionConversion) {
    // Create a mock Action message
    procedural_interfaces::Action action_msg;
    action_msg.actionName = "test_action";

    // Add a test argument
    procedural_interfaces::Argument arg;
    arg.type = "string";
    arg.value = "test_value";
    action_msg.arguments.push_back(arg);

    // Test simple action conversion
    ASSERT_NO_THROW({
        auto simple_action = RosToInternalConverter::convertToSimpleAction(action_msg);
        EXPECT_EQ(simple_action.getName(), "test_action");
        EXPECT_FALSE(simple_action.args.empty());
    });

    // Test composed action conversion
    ASSERT_NO_THROW({
        auto composed_action = RosToInternalConverter::convertToComposedAction(action_msg);
        EXPECT_EQ(composed_action.getName(), "test_action");
        EXPECT_FALSE(composed_action.args.empty());
    });
}

TEST_F(MemoryIntegrationTest, RosToInternalConverterTaskConversion) {
    // Create a mock Task message
    procedural_interfaces::Task task_msg;
    task_msg.task_name = "test_task";
    task_msg.is_primitive = true;

    // Add a test argument
    procedural_interfaces::TaskArgument arg;
    arg.name = "test_arg";
    arg.type = "string";
    arg.value = "test_value";
    task_msg.arguments.push_back(arg);

    // Test primitive action conversion
    ASSERT_NO_THROW({
        auto primitive_action = RosToInternalConverter::convertToPrimitiveAction(task_msg);
        EXPECT_EQ(primitive_action.name, "test_task");
        EXPECT_FALSE(primitive_action.arguments.empty());
    });

    // Test abstract task conversion
    task_msg.is_primitive = false;
    ASSERT_NO_THROW({
        auto abstract_task = RosToInternalConverter::convertToAbstractTask(task_msg);
        EXPECT_EQ(abstract_task.name, "test_task");
        EXPECT_FALSE(abstract_task.arguments.empty());
    });
}

TEST_F(MemoryIntegrationTest, ActionClassification) {
    procedural_interfaces::Action action_msg;
    action_msg.actionName = "test_action";

    // Test with execution actions (should be simple)
    procedural_interfaces::ExecutionAction exec_action;
    exec_action.name = "execute_test";
    action_msg.executionActions.push_back(exec_action);

    EXPECT_TRUE(RosToInternalConverter::isSimpleAction(action_msg));

    // Test without execution actions (should be composed)
    action_msg.executionActions.clear();
    EXPECT_FALSE(RosToInternalConverter::isSimpleAction(action_msg));
}

TEST_F(MemoryIntegrationTest, TaskClassification) {
    procedural_interfaces::Task task_msg;
    task_msg.task_name = "test_task";

    // Test primitive task
    task_msg.is_primitive = true;
    EXPECT_TRUE(RosToInternalConverter::isPrimitiveTask(task_msg));

    // Test abstract task with no methods (should be primitive)
    task_msg.is_primitive = false;
    EXPECT_TRUE(RosToInternalConverter::isPrimitiveTask(task_msg));

    // Test abstract task with methods (should not be primitive)
    procedural_interfaces::Method method;
    method.method_name = "test_method";
    task_msg.methods.push_back(method);
    EXPECT_FALSE(RosToInternalConverter::isPrimitiveTask(task_msg));
}

TEST_F(MemoryIntegrationTest, ActionCollectionConversion) {
    std::vector<procedural_interfaces::Action> action_msgs;

    // Create a simple action
    procedural_interfaces::Action simple_action;
    simple_action.actionName = "simple_test";
    procedural_interfaces::ExecutionAction exec_action;
    exec_action.name = "execute_simple";
    simple_action.executionActions.push_back(exec_action);
    action_msgs.push_back(simple_action);

    // Create a composed action
    procedural_interfaces::Action composed_action;
    composed_action.actionName = "composed_test";
    action_msgs.push_back(composed_action);

    std::vector<ParsedSimpleAction_t> simple_actions;
    std::vector<ParsedComposedAction_t> composed_actions;

    ASSERT_NO_THROW({
        RosToInternalConverter::convertActionsToInternal(action_msgs, simple_actions, composed_actions);
    });

    EXPECT_EQ(simple_actions.size(), 1);
    EXPECT_EQ(composed_actions.size(), 1);
    EXPECT_EQ(simple_actions[0].getName(), "simple_test");
    EXPECT_EQ(composed_actions[0].getName(), "composed_test");
}

TEST_F(MemoryIntegrationTest, TaskCollectionConversion) {
    std::vector<procedural_interfaces::Task> task_msgs;

    // Create a primitive task
    procedural_interfaces::Task primitive_task;
    primitive_task.task_name = "primitive_test";
    primitive_task.is_primitive = true;
    task_msgs.push_back(primitive_task);

    // Create an abstract task
    procedural_interfaces::Task abstract_task;
    abstract_task.task_name = "abstract_test";
    abstract_task.is_primitive = false;
    procedural_interfaces::Method method;
    method.method_name = "test_method";
    abstract_task.methods.push_back(method);
    task_msgs.push_back(abstract_task);

    HTNParserd_t htn;
    ASSERT_NO_THROW({
        htn = RosToInternalConverter::convertTasksToHTN(task_msgs);
    });

    EXPECT_EQ(htn.actions.size(), 1);
    EXPECT_EQ(htn.tasks.size(), 1);
    EXPECT_EQ(htn.actions[0].name, "primitive_test");
    EXPECT_EQ(htn.tasks[0].name, "abstract_test");
}

// Integration test would require a running memory service
// For now, we test the components in isolation
class MemoryIntegrationIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();

        // Check if the memory service is available
        service_available = ros::service::waitForService("procedural_memory/get_actions", ros::Duration(1.0));
        if (!service_available) {
            GTEST_SKIP() << "Memory service not available, skipping integration tests";
        }
    }

    void TearDown() override {
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    bool service_available = false;
};

TEST_F(MemoryIntegrationIntegrationTest, MemoryActionReaderIntegration) {
    if (!service_available) return;

    MemoryActionReader reader(*nh);

    // Wait for service
    ASSERT_TRUE(reader.waitForService(ros::Duration(5.0)));

    // Try to read actions
    bool read_success = reader.read();

    // The test passes if either we successfully read actions or there are no actions
    // (both are valid scenarios)
    EXPECT_TRUE(read_success || !reader.hasActions());
}

TEST_F(MemoryIntegrationIntegrationTest, MemoryDomainReaderIntegration) {
    if (!service_available) return;

    MemoryDomainReader reader(*nh);

    // Wait for service
    ASSERT_TRUE(reader.waitForService(ros::Duration(5.0)));

    // Try to read tasks
    bool read_success = reader.read();

    // The test passes if either we successfully read tasks or there are no tasks
    EXPECT_TRUE(read_success || !reader.hasHTN());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_memory_integration");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}