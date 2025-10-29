#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>

#include "procedural/task_recognition/Reader/MemoryDomainReader.h"
#include "procedural/task_recognition/Builder/HTNBuilder.h"
#include "procedural_interfaces/GetTasks.h"

using namespace procedural;

class MockHTNMemoryService {
public:
    MockHTNMemoryService(ros::NodeHandle& nh) : nh_(nh) {
        server_ = nh_.advertiseService("test_htn_memory/get_tasks",
                                     &MockHTNMemoryService::getTasksCallback, this);
    }

    bool getTasksCallback(procedural_interfaces::GetTasks::Request& req,
                         procedural_interfaces::GetTasks::Response& res) {
        // Create comprehensive test tasks for HTNBuilder testing

        // Primitive task 1: move
        procedural_interfaces::Task move_task;
        move_task.task_name = "move";
        move_task.is_primitive = true;
        move_task.description = "Move robot to location";

        procedural_interfaces::TaskArgument move_arg1;
        move_arg1.name = "from";
        move_arg1.type = "location";
        move_task.arguments.push_back(move_arg1);

        procedural_interfaces::TaskArgument move_arg2;
        move_arg2.name = "to";
        move_arg2.type = "location";
        move_task.arguments.push_back(move_arg2);

        procedural_interfaces::TaskEffect move_effect;
        move_effect.subject = "robot";
        move_effect.predicate = "at";
        move_effect.object = "to";
        move_effect.is_add = true;
        move_task.effects.push_back(move_effect);

        res.tasks.tasks.push_back(move_task);

        // Primitive task 2: pick
        procedural_interfaces::Task pick_task;
        pick_task.task_name = "pick";
        pick_task.is_primitive = true;
        pick_task.description = "Pick up object";

        procedural_interfaces::TaskArgument pick_arg;
        pick_arg.name = "object";
        pick_arg.type = "item";
        pick_task.arguments.push_back(pick_arg);

        procedural_interfaces::TaskEffect pick_effect;
        pick_effect.subject = "robot";
        pick_effect.predicate = "holding";
        pick_effect.object = "object";
        pick_effect.is_add = true;
        pick_task.effects.push_back(pick_effect);

        res.tasks.tasks.push_back(pick_task);

        // Abstract task: transport
        procedural_interfaces::Task transport_task;
        transport_task.task_name = "transport";
        transport_task.is_primitive = false;
        transport_task.description = "Transport object from one location to another";

        procedural_interfaces::TaskArgument trans_arg1;
        trans_arg1.name = "object";
        trans_arg1.type = "item";
        transport_task.arguments.push_back(trans_arg1);

        procedural_interfaces::TaskArgument trans_arg2;
        trans_arg2.name = "from_loc";
        trans_arg2.type = "location";
        transport_task.arguments.push_back(trans_arg2);

        procedural_interfaces::TaskArgument trans_arg3;
        trans_arg3.name = "to_loc";
        trans_arg3.type = "location";
        transport_task.arguments.push_back(trans_arg3);

        // Method 1: simple transport
        procedural_interfaces::Method method1;
        method1.method_name = "simple_transport";

        procedural_interfaces::TaskPrecondition precond1;
        precond1.subject = "robot";
        precond1.predicate = "at";
        precond1.object = "from_loc";
        precond1.is_negative = false;
        method1.preconditions.push_back(precond1);

        method1.decomposition.push_back("pick(object)");
        method1.decomposition.push_back("move(from_loc, to_loc)");

        transport_task.methods.push_back(method1);

        // Method 2: complex transport (robot not at source)
        procedural_interfaces::Method method2;
        method2.method_name = "complex_transport";

        procedural_interfaces::TaskPrecondition precond2;
        precond2.subject = "robot";
        precond2.predicate = "at";
        precond2.object = "from_loc";
        precond2.is_negative = true;  // robot NOT at source
        method2.preconditions.push_back(precond2);

        method2.decomposition.push_back("move(current_loc, from_loc)");
        method2.decomposition.push_back("pick(object)");
        method2.decomposition.push_back("move(from_loc, to_loc)");

        transport_task.methods.push_back(method2);

        res.tasks.tasks.push_back(transport_task);

        // Another abstract task: serve_customer
        procedural_interfaces::Task serve_task;
        serve_task.task_name = "serve_customer";
        serve_task.is_primitive = false;
        serve_task.description = "Serve a customer";

        procedural_interfaces::TaskArgument serve_arg1;
        serve_arg1.name = "customer";
        serve_arg1.type = "person";
        serve_task.arguments.push_back(serve_arg1);

        procedural_interfaces::TaskArgument serve_arg2;
        serve_arg2.name = "item";
        serve_arg2.type = "item";
        serve_task.arguments.push_back(serve_arg2);

        procedural_interfaces::Method serve_method;
        serve_method.method_name = "basic_service";
        serve_method.decomposition.push_back("transport(item, kitchen, customer_table)");

        serve_task.methods.push_back(serve_method);

        res.tasks.tasks.push_back(serve_task);

        return true;
    }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer server_;
};

class HTNBuilderWithMemoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<MockHTNMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MockHTNMemoryService> mock_service;
};

TEST_F(HTNBuilderWithMemoryTest, BuildHTNFromMemoryTest) {
    // Create memory reader
    MemoryDomainReader reader(*nh, "test_htn_memory");

    // Wait for service and read tasks
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    // Verify tasks were read
    EXPECT_TRUE(reader.hasHTN());
    EXPECT_EQ(reader.getTaskCount(), 4);

    auto htn = reader.getHTN();
    EXPECT_EQ(htn.actions.size(), 2);  // move, pick (primitive tasks)
    EXPECT_EQ(htn.tasks.size(), 2);    // transport, serve_customer (abstract tasks)

    // Test HTNBuilder with memory-loaded HTN
    ASSERT_NO_THROW({
        HTNBuilder builder;
        ASSERT_TRUE(builder.build(htn));

        auto built_tasks = builder.getTasks();
        EXPECT_EQ(built_tasks.size(), 2);  // Only abstract tasks become Task objects

        // Verify task names
        std::set<std::string> task_names;
        for (const auto& task : built_tasks) {
            task_names.insert(task->getName());
        }

        EXPECT_TRUE(task_names.count("transport") > 0);
        EXPECT_TRUE(task_names.count("serve_customer") > 0);
    });
}

TEST_F(HTNBuilderWithMemoryTest, HTNStructureValidationTest) {
    MemoryDomainReader reader(*nh, "test_htn_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto htn = reader.getHTN();

    // Validate primitive actions structure
    for (const auto& action : htn.actions) {
        EXPECT_FALSE(action.name.empty());
        EXPECT_FALSE(action.arguments.empty());
        // Primitive actions should have effects
    }

    // Validate abstract tasks structure
    for (const auto& task : htn.tasks) {
        EXPECT_FALSE(task.name.empty());
        EXPECT_FALSE(task.arguments.empty());
        EXPECT_FALSE(task.methods_.empty());

        // Validate methods
        for (const auto& method : task.methods_) {
            EXPECT_FALSE(method.name.empty());
            EXPECT_FALSE(method.subtask.map_actions.empty());
        }
    }
}

TEST_F(HTNBuilderWithMemoryTest, HTNBuilderConsistencyTest) {
    // Test that HTNBuilder produces consistent results
    MemoryDomainReader reader(*nh, "test_htn_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto htn = reader.getHTN();

    // Build HTN first time
    HTNBuilder builder1;
    ASSERT_TRUE(builder1.build(htn));
    auto tasks1 = builder1.getTasks();

    // Build HTN second time
    HTNBuilder builder2;
    ASSERT_TRUE(builder2.build(htn));
    auto tasks2 = builder2.getTasks();

    // Should produce same number of tasks
    EXPECT_EQ(tasks1.size(), tasks2.size());

    // Verify all task names match
    std::set<std::string> names1, names2;
    for (const auto& task : tasks1) names1.insert(task->getName());
    for (const auto& task : tasks2) names2.insert(task->getName());

    EXPECT_EQ(names1, names2);
}

TEST_F(HTNBuilderWithMemoryTest, ComplexMethodTest) {
    MemoryDomainReader reader(*nh, "test_htn_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto htn = reader.getHTN();

    // Find the transport task which has multiple methods
    const Abstract_task_t* transport_task = nullptr;
    for (const auto& task : htn.tasks) {
        if (task.name == "transport") {
            transport_task = &task;
            break;
        }
    }

    ASSERT_NE(transport_task, nullptr);
    EXPECT_EQ(transport_task->methods_.size(), 2);

    // Check method 1: simple_transport
    auto& method1 = transport_task->methods_[0];
    EXPECT_EQ(method1.name, "simple_transport");
    EXPECT_EQ(method1.preconditions.size(), 1);
    EXPECT_EQ(method1.subtask.map_actions.size(), 2);

    // Check method 2: complex_transport
    auto& method2 = transport_task->methods_[1];
    EXPECT_EQ(method2.name, "complex_transport");
    EXPECT_EQ(method2.preconditions.size(), 1);
    EXPECT_EQ(method2.subtask.map_actions.size(), 3);

    // Verify precondition difference
    EXPECT_FALSE(method1.preconditions[0].add);  // robot at from_loc
    EXPECT_FALSE(method2.preconditions[0].add);  // robot NOT at from_loc (is_negative=true)
}

TEST_F(HTNBuilderWithMemoryTest, MethodDecompositionParsingTest) {
    MemoryDomainReader reader(*nh, "test_htn_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto htn = reader.getHTN();

    // Find transport task
    const Abstract_task_t* transport_task = nullptr;
    for (const auto& task : htn.tasks) {
        if (task.name == "transport") {
            transport_task = &task;
            break;
        }
    }

    ASSERT_NE(transport_task, nullptr);

    // Check simple_transport method decomposition parsing
    auto& method = transport_task->methods_[0];
    EXPECT_EQ(method.subtask.map_actions.size(), 2);

    auto action_iter = method.subtask.map_actions.begin();

    // First action: pick(object)
    auto& action1 = action_iter->second;
    EXPECT_EQ(action1.name, "pick");
    EXPECT_EQ(action1.arguments.size(), 1);
    EXPECT_EQ(action1.arguments[0], "object");

    // Second action: move(from_loc, to_loc)
    ++action_iter;
    auto& action2 = action_iter->second;
    EXPECT_EQ(action2.name, "move");
    EXPECT_EQ(action2.arguments.size(), 2);
    EXPECT_EQ(action2.arguments[0], "from_loc");
    EXPECT_EQ(action2.arguments[1], "to_loc");
}

TEST_F(HTNBuilderWithMemoryTest, EmptyHTNTest) {
    // Test HTNBuilder behavior with empty HTN
    HTNParserd_t empty_htn;

    HTNBuilder builder;
    EXPECT_TRUE(builder.build(empty_htn));  // Should succeed but build nothing
    auto built_tasks = builder.getTasks();
    EXPECT_EQ(built_tasks.size(), 0);
}

TEST_F(HTNBuilderWithMemoryTest, NestedTaskTest) {
    MemoryDomainReader reader(*nh, "test_htn_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto htn = reader.getHTN();

    // Find serve_customer task which uses transport task
    const Abstract_task_t* serve_task = nullptr;
    for (const auto& task : htn.tasks) {
        if (task.name == "serve_customer") {
            serve_task = &task;
            break;
        }
    }

    ASSERT_NE(serve_task, nullptr);
    ASSERT_FALSE(serve_task->methods_.empty());

    auto& method = serve_task->methods_[0];
    EXPECT_EQ(method.subtask.map_actions.size(), 1);

    auto& nested_action = method.subtask.map_actions.begin()->second;
    EXPECT_EQ(nested_action.name, "transport");
    EXPECT_EQ(nested_action.arguments.size(), 3);
}

TEST_F(HTNBuilderWithMemoryTest, MemoryReaderReuseWithHTNTest) {
    // Test that MemoryDomainReader can be reused
    MemoryDomainReader reader(*nh, "test_htn_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read multiple times
    ASSERT_TRUE(reader.read());
    size_t first_count = reader.getTaskCount();

    ASSERT_TRUE(reader.read());
    size_t second_count = reader.getTaskCount();

    EXPECT_EQ(first_count, second_count);

    // HTNBuilder should work with any read
    auto htn = reader.getHTN();
    HTNBuilder builder;
    ASSERT_TRUE(builder.build(htn));
    EXPECT_GT(builder.getTasks().size(), 0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_htn_builder_with_memory");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}