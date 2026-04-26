// test_VariablesTable.cpp
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "procedural/structures/VariablesTable.h"
#include <memory>

TEST(VariableTableTest, testVariableTable)
{
    procedural::VariableTable_t table;
    // Use single-argument constructor (type only)
    procedural::Variable_t var("type");
    auto var_ptr = std::make_shared<procedural::Variable_t>(var);
    table.variables["test_var"] = var_ptr;
    table.agents.insert("test_agent");

    ASSERT_EQ(table.variables["test_var"], var_ptr);
    ASSERT_NE(table.agents.find("test_agent"), table.agents.end());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_VariablesTable");
    return RUN_ALL_TESTS();
}