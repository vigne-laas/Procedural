#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_server.h>

#include "procedural/task_recognition/Reader/MemoryDomainReader.h"
#include "procedural_interfaces/GetTasks.h"

using namespace procedural;

class MockTaskMemoryService {
public:
    MockTaskMemoryService(ros::NodeHandle& nh) : nh_(nh) {
        server_ = nh_.advertiseService("procedural_memory/get_tasks",
                                     &MockTaskMemoryService::getTasksCallback, this);
    }

    bool getTasksCallback(procedural_interfaces::GetTasks::Request& req,
                         procedural_interfaces::GetTasks::Response& res) {
        // Mock response with test tasks

        // Create a primitive task
        procedural_interfaces::Task primitive_task;
        primitive_task.task_name = "test_primitive_task";
        primitive_task.is_primitive = true;
        primitive_task.description = "A test primitive task";

        // Add arguments
        procedural_interfaces::TaskArgument arg1;
        arg1.name = "object";
        arg1.type = "string";
        arg1.value = "test_object";
        primitive_task.arguments.push_back(arg1);

        // Add effects
        procedural_interfaces::TaskEffect effect;
        effect.subject = "robot";
        effect.predicate = "has";
        effect.object = "object";
        effect.is_add = true;
        primitive_task.effects.push_back(effect);

        res.tasks.tasks.push_back(primitive_task);

        // Create an abstract task
        procedural_interfaces::Task abstract_task;
        abstract_task.task_name = "test_abstract_task";
        abstract_task.is_primitive = false;
        abstract_task.description = "A test abstract task";

        // Add method
        procedural_interfaces::Method method;
        method.method_name = "test_method";

        // Add preconditions to method
        procedural_interfaces::TaskPrecondition precond;
        precond.subject = "robot";
        precond.predicate = "at";
        precond.object = "location";
        precond.is_negative = false;
        method.preconditions.push_back(precond);

        // Add decomposition
        method.decomposition.push_back("primitive_action(arg1, arg2)");
        method.decomposition.push_back("another_action(arg3)");

        abstract_task.methods.push_back(method);

        res.tasks.tasks.push_back(abstract_task);

        return true;
    }

private:
    ros::NodeHandle& nh_;
    ros::ServiceServer server_;
};

class MemoryDomainReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh = std::make_unique<ros::NodeHandle>();
        mock_service = std::make_unique<MockTaskMemoryService>(*nh);

        // Give the service time to register
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    void TearDown() override {
        mock_service.reset();
        nh.reset();
    }

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MockTaskMemoryService> mock_service;
};

TEST_F(MemoryDomainReaderTest, ConstructorTest) {
    ASSERT_NO_THROW({
        MemoryDomainReader reader(*nh, "procedural_memory");
    });
}

TEST_F(MemoryDomainReaderTest, ConstructorWithFilterTest) {
    ASSERT_NO_THROW({
        MemoryDomainReader reader(*nh, "test_filter", "procedural_memory");
    });
}

TEST_F(MemoryDomainReaderTest, ServiceAvailabilityTest) {
    MemoryDomainReader reader(*nh, "procedural_memory");

    // Wait for service to be available
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    EXPECT_TRUE(reader.isServiceAvailable());
}

TEST_F(MemoryDomainReaderTest, ReadTasksTest) {
    MemoryDomainReader reader(*nh, "procedural_memory");

    // Wait for service
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read tasks
    EXPECT_TRUE(reader.read());
    EXPECT_TRUE(reader.hasHTN());
    EXPECT_EQ(reader.getTaskCount(), 2);

    // Check HTN structure
    auto htn = reader.getHTN();
    EXPECT_EQ(htn.actions.size(), 1);  // Primitive task becomes action
    EXPECT_EQ(htn.tasks.size(), 1);    // Abstract task stays as task

    // Check primitive action
    EXPECT_EQ(htn.actions[0].name, "test_primitive_task");
    EXPECT_EQ(htn.actions[0].arguments.size(), 1);
    EXPECT_EQ(htn.actions[0].arguments[0].name, "object");
    EXPECT_EQ(htn.actions[0].effects.simple_effects.size(), 1);

    // Check abstract task
    EXPECT_EQ(htn.tasks[0].name, "test_abstract_task");
    EXPECT_EQ(htn.tasks[0].methods_.size(), 1);
    EXPECT_EQ(htn.tasks[0].methods_[0].name, "test_method");
    EXPECT_EQ(htn.tasks[0].methods_[0].preconditions.size(), 1);
    EXPECT_EQ(htn.tasks[0].methods_[0].subtask.map_actions.size(), 2); // Two decomposition actions
}

TEST_F(MemoryDomainReaderTest, ReadWithFilterTest) {
    MemoryDomainReader reader(*nh, "procedural_memory");

    // Wait for service
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Read with filter
    EXPECT_TRUE(reader.read("test_filter"));
    EXPECT_TRUE(reader.hasHTN());
}

TEST_F(MemoryDomainReaderTest, ConstructorWithAutoReadTest) {
    // Wait for service to be available first
    MemoryDomainReader temp_reader(*nh, "procedural_memory");
    ASSERT_TRUE(temp_reader.waitForService(ros::Duration(2.0)));

    // Test constructor with auto-read
    ASSERT_NO_THROW({
        MemoryDomainReader reader(*nh, "", "procedural_memory");
        EXPECT_TRUE(reader.hasHTN());
        EXPECT_GT(reader.getTaskCount(), 0);
    });
}

TEST_F(MemoryDomainReaderTest, NoServiceTest) {
    // Create reader with non-existent service namespace
    MemoryDomainReader reader(*nh, "non_existent_service");

    EXPECT_FALSE(reader.isServiceAvailable());
    EXPECT_FALSE(reader.waitForService(ros::Duration(1.0)));
    EXPECT_FALSE(reader.read());
    EXPECT_FALSE(reader.hasHTN());
    EXPECT_EQ(reader.getTaskCount(), 0);
}

TEST_F(MemoryDomainReaderTest, EmptyResponseTest) {
    MemoryDomainReader reader(*nh, "procedural_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));

    // Even with our mock response, the reader should handle it gracefully
    EXPECT_TRUE(reader.read());

    // Verify the HTN structure is valid
    auto htn = reader.getHTN();
    EXPECT_FALSE(htn.empty());
    EXPECT_EQ(htn.actions.size() + htn.tasks.size(), reader.getTaskCount());
}

TEST_F(MemoryDomainReaderTest, HTNConversionTest) {
    MemoryDomainReader reader(*nh, "procedural_memory");
    ASSERT_TRUE(reader.waitForService(ros::Duration(2.0)));
    ASSERT_TRUE(reader.read());

    auto htn = reader.getHTN();

    // Test primitive action conversion
    ASSERT_GT(htn.actions.size(), 0);
    auto& primitive_action = htn.actions[0];
    EXPECT_EQ(primitive_action.name, "test_primitive_task");
    EXPECT_FALSE(primitive_action.arguments.empty());
    EXPECT_FALSE(primitive_action.effects.simple_effects.empty());

    // Test abstract task conversion
    ASSERT_GT(htn.tasks.size(), 0);
    auto& abstract_task = htn.tasks[0];
    EXPECT_EQ(abstract_task.name, "test_abstract_task");
    EXPECT_FALSE(abstract_task.methods_.empty());

    // Test method conversion
    auto& method = abstract_task.methods_[0];
    EXPECT_EQ(method.name, "test_method");
    EXPECT_FALSE(method.preconditions.empty());
    EXPECT_FALSE(method.subtask.map_actions.empty());

    // Test subtask decomposition parsing
    EXPECT_EQ(method.subtask.map_actions.size(), 2);
    auto& first_action = method.subtask.map_actions.begin()->second;
    EXPECT_EQ(first_action.name, "primitive_action");
    EXPECT_EQ(first_action.arguments.size(), 2);
    EXPECT_EQ(first_action.arguments[0], "arg1");
    EXPECT_EQ(first_action.arguments[1], "arg2");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_memory_domain_reader");

    ::testing::InitGoogleTest(&argc, argv);

    // Start spinning in a separate thread to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return result;
}