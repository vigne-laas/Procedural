#include <gtest/gtest.h>
#include <ros/ros.h>
#include "procedural/structures/graph/Node.h"
#include "procedural/structures/graph/Transition.h"
#include "procedural/structures/Observation.h"

TEST(NodeTest, matchTest)
{
    // Create a Node instance
    procedural::Node node(1, "test_node");

    // Create an Observation instance
    procedural::Observation observation(1);

    // Create a Transition instance
    std::shared_ptr<procedural::Transition> transition = std::make_shared<procedural::Transition>(1, &observation, 2, 1);

    // Add the transition to the node
    node.addTransition(transition);

    // Test the match function
    uint64_t match_result = node.match(&observation);

    // Check if the match function returns the correct result
    EXPECT_EQ(match_result, 2);
}


TEST(NodeTest, getIdTest)
{
    // Create a Node instance
    procedural::Node node(1, "test_node");

    // Test the getId function
    uint64_t id = node.getId();

    // Check if the getId function returns the correct result
    EXPECT_EQ(id, 1);
}

TEST(NodeTest, getFullNameTest)
{
    // Create a Node instance
    procedural::Node node(1, "test_node");

    // Test the getFullName function
    std::string full_name = node.getFullName();

    // Check if the getFullName function returns the correct result
    EXPECT_EQ(full_name, "test_node_1");
}

TEST(NodeTest, addParentsTest)
{
    // Create a Node instance
    procedural::Node node(1, "test_node");

    // Add a parent to the node
    node.addParent(2);

    // Test the getTransitions function
    const std::vector<uint64_t>& parents = node.getParents();

    // Check if the getTransitions function returns the correct result
    EXPECT_EQ(parents.size(), 1);
    EXPECT_EQ(parents[0], 2);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node");
    return RUN_ALL_TESTS();
}