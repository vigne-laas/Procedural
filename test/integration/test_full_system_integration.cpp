#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <thread>
#include <chrono>

#include "procedural/memory_client/MemoryServiceClient.h"
#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural/task_recognition/Reader/MemoryDomainReader.h"
#include "procedural/action_recognition/builder/ActionBuilder.h"
#include "procedural/task_recognition/Builder/HTNBuilder.h"
#include "procedural_interfaces/GetActions.h"
#include "procedural_interfaces/GetTasks.h"

using namespace procedural;

class FullSystemMockMemoryService {
public:
    FullSystemMockMemoryService(ros::NodeHandle& nh) : nh_(nh) {
        actions_server_ = nh_.advertiseService("full_system_memory/get_actions",
                                              &FullSystemMockMemoryService::getActionsCallback, this);
        tasks_server_ = nh_.advertiseService("full_system_memory/get_tasks",
                                            &FullSystemMockMemoryService::getTasksCallback, this);
    }

    bool getActionsCallback(procedural_interfaces::GetActions::Request& req,
                           procedural_interfaces::GetActions::Response& res) {
        // Create comprehensive action set for full system test

        // Simple action 1: move_robot
        procedural_interfaces::Action move_action;
        move_action.actionName = "move_robot";

        procedural_interfaces::ExecutionAction exec_move;
        exec_move.name = "navigate_to_location";
        move_action.executionActions.push_back(exec_move);

        procedural_interfaces::Argument move_arg;
        move_arg.type = "location";
        move_arg.literal = "?target";
        move_arg.value = "target_location";
        move_action.arguments.push_back(move_arg);

        res.actions.push_back(move_action);

        // Simple action 2: grasp_object
        procedural_interfaces::Action grasp_action;
        grasp_action.actionName = "grasp_object";

        procedural_interfaces::ExecutionAction exec_grasp;
        exec_grasp.name = "close_gripper";
        grasp_action.executionActions.push_back(exec_grasp);

        procedural_interfaces::Argument grasp_arg;
        grasp_arg.type = "object";
        grasp_arg.literal = "?obj";
        grasp_arg.value = "target_object";
        grasp_action.arguments.push_back(grasp_arg);

        res.actions.push_back(grasp_action);

        // Composed action: fetch_object
        procedural_interfaces::Action fetch_action;
        fetch_action.actionName = "fetch_object";
        // No execution actions makes it composed

        procedural_interfaces::Argument fetch_arg1;
        fetch_arg1.type = "object";
        fetch_arg1.literal = "?item";
        fetch_arg1.value = "item_to_fetch";
        fetch_action.arguments.push_back(fetch_arg1);

        procedural_interfaces::Argument fetch_arg2;
        fetch_arg2.type = "location";
        fetch_arg2.literal = "?location";
        fetch_arg2.value = "item_location";
        fetch_action.arguments.push_back(fetch_arg2);

        res.actions.push_back(fetch_action);

        return true;
    }

    bool getTasksCallback(procedural_interfaces::GetTasks::Request& req,
                         procedural_interfaces::GetTasks::Response& res) {
        // Create comprehensive task set for full system test

        // Primitive task: navigate
        procedural_interfaces::Task navigate_task;
        navigate_task.task_name = "navigate";
        navigate_task.is_primitive = true;
        navigate_task.description = "Navigate robot to target location";

        procedural_interfaces::TaskArgument nav_arg;
        nav_arg.name = "destination";
        nav_arg.type = "location";
        nav_arg.value = "?dest";
        navigate_task.arguments.push_back(nav_arg);

        procedural_interfaces::TaskEffect nav_effect;
        nav_effect.subject = "robot";
        nav_effect.predicate = "at";
        nav_effect.object = "destination";
        nav_effect.is_add = true;
        navigate_task.effects.push_back(nav_effect);

        res.tasks.tasks.push_back(navigate_task);

        // Abstract task: deliver_item
        procedural_interfaces::Task deliver_task;
        deliver_task.task_name = "deliver_item";
        deliver_task.is_primitive = false;
        deliver_task.description = "Deliver item to customer";

        procedural_interfaces::TaskArgument del_arg1;
        del_arg1.name = "item";
        del_arg1.type = "object";
        del_arg1.value = "?item";
        deliver_task.arguments.push_back(del_arg1);

        procedural_interfaces::TaskArgument del_arg2;
        del_arg2.name = "customer";
        del_arg2.type = "person";
        del_arg2.value = "?customer";
        deliver_task.arguments.push_back(del_arg2);

        // Method for delivery
        procedural_interfaces::Method delivery_method;
        delivery_method.method_name = "standard_delivery";

        procedural_interfaces::TaskPrecondition del_precond;
        del_precond.subject = "robot";
        del_precond.predicate = "has";
        del_precond.object = "item";
        del_precond.is_negative = false;
        delivery_method.preconditions.push_back(del_precond);

        delivery_method.decomposition.push_back("navigate(customer_location)");
        delivery_method.decomposition.push_back("hand_over(item, customer)");

        deliver_task.methods.push_back(delivery_method);

        res.tasks.tasks.push_back(deliver_task);

        // Complex abstract task: serve_customer
        procedural_interfaces::Task serve_task;
        serve_task.task_name = "serve_customer";
        serve_task.is_primitive = false;
        serve_task.description = "Complete customer service";

        procedural_interfaces::TaskArgument serve_arg;
        serve_arg.name = "customer_id";
        serve_arg.type = "string";
        serve_arg.value = "?customer";
        serve_task.arguments.push_back(serve_arg);

        // Method 1: if item is available
        procedural_interfaces::Method serve_method1;
        serve_method1.method_name = "direct_service";

        procedural_interfaces::TaskPrecondition serve_precond1;
        serve_precond1.subject = "item";
        serve_precond1.predicate = "available";
        serve_precond1.object = "true";
        serve_precond1.is_negative = false;
        serve_method1.preconditions.push_back(serve_precond1);

        serve_method1.decomposition.push_back("fetch_item(requested_item)");
        serve_method1.decomposition.push_back("deliver_item(requested_item, customer_id)");

        serve_task.methods.push_back(serve_method1);

        // Method 2: if item needs preparation
        procedural_interfaces::Method serve_method2;
        serve_method2.method_name = "prepare_and_serve";

        procedural_interfaces::TaskPrecondition serve_precond2;
        serve_precond2.subject = "item";
        serve_precond2.predicate = "available";
        serve_precond2.object = "true";
        serve_precond2.is_negative = true;  // item NOT available
        serve_method2.preconditions.push_back(serve_precond2);

        serve_method2.decomposition.push_back("prepare_item(requested_item)");
        serve_method2.decomposition.push_back("deliver_item(requested_item, customer_id)");

        serve_task.methods.push_back(serve_method2);

        res.tasks.tasks.push_back(serve_task);

        return true;
    }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer actions_server_;
    ros::ServiceServer tasks_server_;
};

class FullSystemIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<FullSystemMockMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<FullSystemMockMemoryService> mock_service;
};

TEST_F(FullSystemIntegrationTest, EndToEndActionWorkflowTest) {
    // Test complete workflow: Memory → Reader → Builder → Recognition

    // Step 1: Test MemoryServiceClient
    MemoryServiceClient client(*nh, "full_system_memory");
    ASSERT_TRUE(client.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(client.isServiceAvailable());

    auto actions = client.getActions();
    EXPECT_EQ(actions.size(), 3);  // move_robot, grasp_object, fetch_object

    // Step 2: Test MemoryActionReader
    MemoryActionReader action_reader(*nh, "full_system_memory");
    ASSERT_TRUE(action_reader.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(action_reader.read());

    EXPECT_TRUE(action_reader.hasActions());
    EXPECT_EQ(action_reader.getActionCount(), 3);

    auto simple_actions = action_reader.getSimpleActions();
    auto composed_actions = action_reader.getComposedActions();
    EXPECT_EQ(simple_actions.size(), 2);   // move_robot, grasp_object
    EXPECT_EQ(composed_actions.size(), 1); // fetch_object

    // Step 3: Test ActionBuilder with memory data
    ASSERT_NO_THROW({
        ActionBuilder builder(simple_actions, composed_actions, "/tmp/full_system_test");
        auto built_actions = builder.getActions();
        EXPECT_EQ(built_actions.size(), 3);

        std::set<std::string> action_names;
        for (const auto& action : built_actions) {
            action_names.insert(action->getName());
        }

        EXPECT_TRUE(action_names.count("move_robot") > 0);
        EXPECT_TRUE(action_names.count("grasp_object") > 0);
        EXPECT_TRUE(action_names.count("fetch_object") > 0);
    });
}

TEST_F(FullSystemIntegrationTest, EndToEndTaskWorkflowTest) {
    // Test complete workflow: Memory → Reader → Builder

    // Step 1: Test MemoryServiceClient for tasks
    MemoryServiceClient client(*nh, "full_system_memory");
    ASSERT_TRUE(client.waitForService(ros::Duration(5.0)));

    auto tasks = client.getTasks();
    EXPECT_EQ(tasks.size(), 3);  // navigate, deliver_item, serve_customer

    // Step 2: Test MemoryDomainReader
    MemoryDomainReader domain_reader(*nh, "full_system_memory");
    ASSERT_TRUE(domain_reader.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(domain_reader.read());

    EXPECT_TRUE(domain_reader.hasHTN());
    EXPECT_EQ(domain_reader.getTaskCount(), 3);

    auto htn = domain_reader.getHTN();
    EXPECT_EQ(htn.actions.size(), 1);  // navigate (primitive)
    EXPECT_EQ(htn.tasks.size(), 2);    // deliver_item, serve_customer (abstract)

    // Step 3: Test HTNBuilder with memory data
    ASSERT_NO_THROW({
        HTNBuilder builder;
        ASSERT_TRUE(builder.build(htn));

        auto built_tasks = builder.getTasks();
        EXPECT_EQ(built_tasks.size(), 2);

        std::set<std::string> task_names;
        for (const auto& task : built_tasks) {
            task_names.insert(task->getName());
        }

        EXPECT_TRUE(task_names.count("deliver_item") > 0);
        EXPECT_TRUE(task_names.count("serve_customer") > 0);
    });
}

TEST_F(FullSystemIntegrationTest, CompleteSystemIntegrationTest) {
    // Test both action and task workflows together

    // Initialize both readers
    MemoryActionReader action_reader(*nh, "full_system_memory");
    MemoryDomainReader domain_reader(*nh, "full_system_memory");

    // Wait for services
    ASSERT_TRUE(action_reader.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(domain_reader.waitForService(ros::Duration(5.0)));

    // Read data
    ASSERT_TRUE(action_reader.read());
    ASSERT_TRUE(domain_reader.read());

    // Verify data integrity
    EXPECT_TRUE(action_reader.hasActions());
    EXPECT_TRUE(domain_reader.hasHTN());

    // Build action system
    auto simple_actions = action_reader.getSimpleActions();
    auto composed_actions = action_reader.getComposedActions();

    ActionBuilder action_builder(simple_actions, composed_actions);
    auto built_actions = action_builder.getActions();

    // Build task system
    auto htn = domain_reader.getHTN();
    HTNBuilder task_builder;
    ASSERT_TRUE(task_builder.build(htn));
    auto built_tasks = task_builder.getTasks();

    // Verify both systems are built successfully
    EXPECT_GT(built_actions.size(), 0);
    EXPECT_GT(built_tasks.size(), 0);

    // Verify system coherence
    EXPECT_EQ(built_actions.size(), action_reader.getActionCount());
    EXPECT_EQ(built_tasks.size(), htn.tasks.size());
}

TEST_F(FullSystemIntegrationTest, ConcurrentAccessTest) {
    // Test multiple clients accessing the same memory service

    MemoryServiceClient client1(*nh, "full_system_memory");
    MemoryServiceClient client2(*nh, "full_system_memory");

    ASSERT_TRUE(client1.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(client2.waitForService(ros::Duration(5.0)));

    // Concurrent access to actions
    auto actions1 = client1.getActions();
    auto actions2 = client2.getActions();

    EXPECT_EQ(actions1.size(), actions2.size());
    EXPECT_EQ(actions1.size(), 3);

    // Concurrent access to tasks
    auto tasks1 = client1.getTasks();
    auto tasks2 = client2.getTasks();

    EXPECT_EQ(tasks1.size(), tasks2.size());
    EXPECT_EQ(tasks1.size(), 3);
}

TEST_F(FullSystemIntegrationTest, DataConsistencyTest) {
    // Test that data remains consistent across multiple reads

    MemoryActionReader action_reader(*nh, "full_system_memory");
    MemoryDomainReader domain_reader(*nh, "full_system_memory");

    ASSERT_TRUE(action_reader.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(domain_reader.waitForService(ros::Duration(5.0)));

    // Read multiple times
    ASSERT_TRUE(action_reader.read());
    size_t first_action_count = action_reader.getActionCount();

    ASSERT_TRUE(domain_reader.read());
    size_t first_task_count = domain_reader.getTaskCount();

    // Read again
    ASSERT_TRUE(action_reader.read());
    size_t second_action_count = action_reader.getActionCount();

    ASSERT_TRUE(domain_reader.read());
    size_t second_task_count = domain_reader.getTaskCount();

    // Should be consistent
    EXPECT_EQ(first_action_count, second_action_count);
    EXPECT_EQ(first_task_count, second_task_count);

    // Build systems multiple times for consistency
    ActionBuilder builder1(action_reader.getSimpleActions(), action_reader.getComposedActions());
    auto actions1 = builder1.getActions();

    ActionBuilder builder2(action_reader.getSimpleActions(), action_reader.getComposedActions());
    auto actions2 = builder2.getActions();

    EXPECT_EQ(actions1.size(), actions2.size());
}

TEST_F(FullSystemIntegrationTest, ComplexMethodHandlingTest) {
    // Test that complex methods with multiple preconditions and decompositions work

    MemoryDomainReader domain_reader(*nh, "full_system_memory");
    ASSERT_TRUE(domain_reader.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(domain_reader.read());

    auto htn = domain_reader.getHTN();

    // Find serve_customer task which has multiple methods
    const Abstract_task_t* serve_task = nullptr;
    for (const auto& task : htn.tasks) {
        if (task.name == "serve_customer") {
            serve_task = &task;
            break;
        }
    }

    ASSERT_NE(serve_task, nullptr);
    EXPECT_EQ(serve_task->methods_.size(), 2);

    // Verify both methods were parsed correctly
    for (const auto& method : serve_task->methods_) {
        EXPECT_FALSE(method.name.empty());
        EXPECT_FALSE(method.preconditions.empty());
        EXPECT_FALSE(method.subtask.map_actions.empty());

        // Verify decomposition was parsed
        for (const auto& action_pair : method.subtask.map_actions) {
            const auto& action = action_pair.second;
            EXPECT_FALSE(action.name.empty());
        }
    }

    // Build with HTNBuilder
    HTNBuilder builder;
    ASSERT_TRUE(builder.build(htn));
    auto built_tasks = builder.getTasks();

    // Verify serve_customer task was built
    bool found_serve_task = false;
    for (const auto& task : built_tasks) {
        if (task->getName() == "serve_customer") {
            found_serve_task = true;
            break;
        }
    }
    EXPECT_TRUE(found_serve_task);
}

TEST_F(FullSystemIntegrationTest, ErrorRecoveryTest) {
    // Test system behavior when service becomes unavailable

    MemoryActionReader action_reader(*nh, "full_system_memory");
    ASSERT_TRUE(action_reader.waitForService(ros::Duration(5.0)));
    ASSERT_TRUE(action_reader.read());

    // Service is available and working
    EXPECT_TRUE(action_reader.hasActions());

    // Test with non-existent service
    MemoryActionReader bad_reader(*nh, "non_existent_service");
    EXPECT_FALSE(bad_reader.waitForService(ros::Duration(1.0)));
    EXPECT_FALSE(bad_reader.read());
    EXPECT_FALSE(bad_reader.hasActions());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_full_system_integration");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(2);  // Use 2 threads for more complex testing
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}