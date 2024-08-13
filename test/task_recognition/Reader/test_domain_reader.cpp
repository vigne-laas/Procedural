#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "procedural/task_recognition/Reader/DomainReader.h"
#include "procedural/utils/Logger.h"

namespace procedural {
  TEST(DomainReaderTest, testRead)
  {
    // Create an instance of DomainReader
    DomainReader domainReader;

    // Call the read method with a test file path
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/task_recognition/Reader/test_domain.dom";
    bool readResult = domainReader.read(file_path);

    // Check the result of the read method
    ASSERT_TRUE(readResult);

    // Get the methods and actions
    std::vector<Abstract_task_t> tasks = domainReader.getTasks();
    std::vector<PrimitiveActionParsed_t> actions = domainReader.getActions();
    // Check the size of the methods and actions vectors
    ASSERT_EQ(tasks.size(), 2);
    // Loop through tasks and check their properties
    for(const auto& task : tasks)
    {
      // Replace "TaskName" with your expected task name
      if(task.name == "Place")
      {
        ASSERT_EQ(task.name, "Place");
        // Check the size of the goals, arguments, and methods
        ASSERT_EQ(task.goals.size(), 1);
        auto goal = task.goals.front();
        LOG_INFO << "Goal: " << goal;
        ASSERT_EQ(goal.subject, "O.isOn");
        ASSERT_EQ(goal.property, "==");
        ASSERT_EQ(goal.object, "T");
        ASSERT_EQ(task.arguments.size(), 3); // Replace with your expected number of arguments
        for(const auto& arg : task.arguments)
        {
          if(arg.type == "Pickable")
            ASSERT_EQ(arg.varname, "O");
          else if(arg.type == "Table")
            ASSERT_EQ(arg.varname, "T");
          else if(arg.type == "Agent")
            ASSERT_EQ(arg.varname, "A");
          else
          {
            std::cerr << "Argument type not recognized";
            ASSERT_TRUE(false);
          }
        }
        ASSERT_EQ(task.methods_.size(), 1); // Replace with your expected number of methods
        auto method = task.methods_.front();
        ASSERT_EQ(method.preconditions.size(), 0);
        auto subtask = method.subtask;
        ASSERT_EQ(subtask.selections.size(), 0);
        ASSERT_EQ(subtask.map_actions.size(), 2);
        auto action = (subtask.map_actions.begin())->second;
        ASSERT_EQ(action.name, "Get");
        ASSERT_EQ(action.id, 1);
        ASSERT_EQ(action.arguments.size(), 2);
        ASSERT_EQ(action.arguments.front(), "A");
        ASSERT_EQ(action.arguments.back(), "O");
        ASSERT_EQ(action.after_id.size(), 0);
        action = subtask.map_actions.at(2);
        ASSERT_EQ(action.name, "Place");
        ASSERT_EQ(action.id, 2);
        ASSERT_EQ(action.arguments.size(), 3);
        ASSERT_EQ(action.arguments.front(), "A");
        ASSERT_EQ(action.arguments.at(1), "O");
        ASSERT_EQ(action.arguments.back(), "T");
        ASSERT_EQ(action.after_id.size(), 1);
        ASSERT_EQ(*action.after_id.begin(), 1);
      }
      else if(task.name == "Get")
      {
        ASSERT_EQ(task.name, "Get");
        auto goal = task.goals.front();
        //            std::cout << "Goal: " << goal;
        ASSERT_EQ(goal.subject, "A.hasInRightHand");
        ASSERT_EQ(goal.property, "==");
        ASSERT_EQ(goal.object, "O");
        // Check the size of the goals, arguments, and methods
        //            ASSERT_EQ(task.goals.size(), 1);  // Replace with your expected number of goals
        //            ASSERT_EQ(task.arguments.size(), 1);  // Replace with your expected number of arguments
        //            ASSERT_EQ(task.methods_.size(), 1);  // Replace with your expected number of methods
      }
      else
      {
        if(task.name != "Test_Partial_Order")
        {
          std::cerr << "Task name not recognized";
          ASSERT_TRUE(false);
        }
      }
    }

    // Loop through actions and check their properties

    ASSERT_EQ(actions.size(), 3);
    for(const auto& action : actions)
    {
      if(action.name == "Pick")
      {
        ASSERT_EQ(action.name, "Pick");
        ASSERT_EQ(action.arguments.size(), 2);
        ASSERT_EQ(action.arguments.front().varname, "A");
        ASSERT_EQ(action.arguments.front().type, "Agent");
        ASSERT_EQ(action.arguments.back().varname, "O");
        ASSERT_EQ(action.arguments.back().type, "Pickable");
      }
      else if(action.name == "Place")
      {
        ASSERT_EQ(action.name, "Place");
        ASSERT_EQ(action.arguments.size(), 3);
        ASSERT_EQ(action.arguments.front().varname, "A");
        ASSERT_EQ(action.arguments.front().type, "Agent");
        ASSERT_EQ(action.arguments.at(1).type, "Pickable");
        ASSERT_EQ(action.arguments.at(1).varname, "O");
        ASSERT_EQ(action.arguments.back().type, "Object");
        ASSERT_EQ(action.arguments.back().varname, "S");
      }
      else if(action.name == "Give")
      {
        ASSERT_EQ(action.name, "Give");
        ASSERT_EQ(action.arguments.size(), 3);
        ASSERT_EQ(action.arguments.front().varname, "A1");
        ASSERT_EQ(action.arguments.front().type, "Agent");
        ASSERT_EQ(action.arguments.back().varname, "A2");
        ASSERT_EQ(action.arguments.back().type, "Agent");
        ASSERT_EQ(action.arguments.at(1).type, "Pickable");
        ASSERT_EQ(action.arguments.at(1).varname, "O");
      }
      else
      {
        std::cerr << "Action name not recognized";
        ASSERT_TRUE(false);
      }
    }
  }

} // namespace procedural

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "domain_reader_test");
  return RUN_ALL_TESTS();
}