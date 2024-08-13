#include <ros/package.h>

#include "gtest/gtest.h"
#include "procedural/task_recognition/Builder/HTNBuilder.h"
#include "procedural/task_recognition/Reader/DomainReader.h"

namespace procedural {

  class HTNBuilderPartialOrderTest : public ::testing::Test
  {
  protected:
    DomainReader reader;
    HTNBuilder builder;

    void SetUp() override
    {
      // Initialisez le reader avec le fichier de données
      std::string package_path = ros::package::getPath("procedural");
      //      /home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/old/tests/kitchen_domain/task.dom
      std::string file_path = package_path + "/test/task_recognition/Builder/test_domain_partial_order.dom";
      //      std::string file_path = package_path + "src/old/tests/kitchen_domain/task.dom";
      bool readResult = reader.read(file_path);
      assert(readResult);
    }

    void TearDown() override
    {
      // Nettoyez les ressources si nécessaire
    }
  };

  class HTNBuilderTest : public ::testing::Test
  {
  protected:
    DomainReader reader;
    HTNBuilder builder;

    void SetUp() override
    {
      //       Initialisez le reader avec le fichier de données
      std::string package_path = ros::package::getPath("procedural");
      std::string file_path = package_path + "/test/task_recognition/Reader/test_domain.dom";
      //      std::string file_path = package_path + "src/old/tests/kitchen_domain/task.dom";

      bool readResult = reader.read(file_path);
      assert(readResult);
      //            std::string file_action_path = package_path + "/test/task_recognition/Reader/test_action.yaml";
      //            bool readActionResult = reader.readActions(file_action_path);
    }

    void TearDown() override
    {
      // Nettoyez les ressources si nécessaire
    }
  };

  TEST_F(HTNBuilderPartialOrderTest, TestBuild)
  {
    builder.build(reader.getHTN());
    auto tasks = builder.getTasks();
    ASSERT_EQ(tasks.size(), 1);
    auto task = tasks[0];
    ASSERT_EQ(task->getName(), "Test_Partial_Order");
    auto methods = task->getFactoryMethods();
    ASSERT_EQ(methods.size(), 4);
  }

  TEST_F(HTNBuilderPartialOrderTest, TestMethod0_2PartialOrder)
  {
    builder.build(reader.getHTN());
    auto method = builder.getTasks()[0]->getFactoryMethods()[0];
    LOG_INFO << "Method: " << method->getName();
    ASSERT_EQ(method->getName(), "Test_Partial_Order_m_0");
    ASSERT_EQ(method->getState(), GraphState::Closed);
    auto initial_node = method->getInitialNode();
    ASSERT_EQ(initial_node->getFullName(), "Test_Partial_Order_m_0_0");
    ASSERT_EQ(initial_node->getDepth(), 0);
    ASSERT_EQ(initial_node->getTransitions().size(), 2);
    auto task = builder.getTasks()[0];
    LOG_INFO << "task: " << task->getName();

    for(auto action : WordTable::actions_table)
    {
      LOG_INFO << "action disponible : " << action;
    }
    auto var1 = std::make_shared<Variable_t>("unset");
    var1->value_ = 1;
    auto var2 = std::make_shared<Variable_t>("unset");
    var2->value_ = 2;
    auto var3 = std::make_shared<Variable_t>("unset");
    var3->value_ = 3;
    VariableTable_t variables_table1;
    variables_table1.variables["A"] = var1;
    variables_table1.variables["C1"] = var2;
    auto* obs = new Observation(WordTable::actions_table.get("Y"), variables_table1);

    if(task->evolve(obs))
    {
      LOG_INFO << "evolve";
    }
    else
    {
      LOG_INFO << "no evolve";
    }
    auto active_methods = task->getActiveMethods();
    ASSERT_EQ(active_methods.size(), 1);

    VariableTable_t variables_table2;
    variables_table2.variables["A"] = var1;
    variables_table2.variables["C2"] = var3;
    auto* obs2 = new Observation(WordTable::actions_table.get("Z"), variables_table2);
    if(task->evolve(obs2))
    {
      LOG_INFO << "evolve";
    }
    else
    {
      LOG_INFO << "no evolve";
    }
    active_methods = task->getActiveMethods();
    ASSERT_EQ(active_methods.size(), 0);
    auto finished_methods = task->getFinishedMethods();
    ASSERT_EQ(finished_methods.size(), 1);
  }

} // namespace procedural

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}