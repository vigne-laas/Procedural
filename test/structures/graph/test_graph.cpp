#include <gtest/gtest.h>
#include <ros/ros.h>
#include "procedural/structures/graph/Graph.h"
#include "procedural/structures/graph/Node.h"
#include "procedural/structures/graph/Transition.h"
#include "procedural/structures/Observation.h"

TEST(GraphTest, addTransitionTest)
{
    // Create a Graph instance
    procedural::Graph graph("graph", 1, "type");
    // Create a table of variables
    procedural::VariableTable_t table;
    // Create a Variable instance
    auto var = std::make_shared<procedural::Variable_t>("A","var");
    // Add the variable to the table
    table.variables.insert(std::make_pair("A", var));
    table.agents.insert("agent1");

    // Create an Observation instance
    procedural::Observation observation(1);
    // Create a Transition instance
    auto transition = std::make_shared<procedural::Transition>(1, &observation, 2, 1);
    // Test the addTransition function
    graph.addTransition(transition);
    auto nodes = graph.getNodes();
    EXPECT_EQ(nodes.size(), 2);
    auto node = nodes[1];
    EXPECT_EQ(node->getTransitions().size(), 1);
    node = nodes[2];
    EXPECT_EQ(node->getTransitions().size(), 0);


}

TEST(GraphTest, closeFalseTest)
{
    // Create a Graph instance
    procedural::Graph graph("test_graph", 1, "test_type");
    // Test the close function
    bool result;
    try{
        result = graph.close();
        FAIL() << "Expected prodecural::Graph::close to throw an exception";
    }
    catch (procedural::NoInitialNodeGraphException& e){
        EXPECT_EQ(e.what(), std::string("Invalid State Machine due to no initial State detected"));
    }
    catch (...){
        FAIL() << "Expected procedural::Graph::close to throw a NoInitialNodeGraphException";
    }

    // Check if the close function returns the correct result
    EXPECT_FALSE(result);
}


TEST(GraphTest, closeTest)
{
    // Create a Graph instance
    procedural::Graph graph("test_graph", 1, "test_type");
    // Create a table of variables
    procedural::VariableTable_t table;
    // Create a Variable instance
    auto var = std::make_shared<procedural::Variable_t>("A","var");
    // Add the variable to the table
    table.variables.insert(std::make_pair("A", var));
    table.agents.insert("agent1");

    // Create an Observation instance
    procedural::Observation observation(1);
    // Create a Transition instance
    auto transition = std::make_shared<procedural::Transition>(1, &observation, 2, 1);
    // Test the addTransition function
    graph.addTransition(transition);
    // Test the close function
    bool result = graph.close();
    // Check if the close function returns the correct result
    EXPECT_TRUE(result);
    EXPECT_EQ(graph.getState(), procedural::GraphState::Closed);
    // verify initial and final state
    auto inital_node = graph.getInitialNode();
    auto final_node = graph.getFinalNode();
    EXPECT_EQ(inital_node->getId(), 1);
    EXPECT_EQ(final_node->getId(), 2);

}


TEST(GraphTest, evolveTest)
{
    // Create a Graph instance
    procedural::Graph graph("test_graph", 1, "test_type");
    // Create an Observation instance
    procedural::Observation observation(1);
    // Create a Transition instance
    std::shared_ptr<procedural::Transition> transition = std::make_shared<procedural::Transition>(1, &observation, 2,1);
    graph.addTransition(transition);

    // Add the transition to the graph
    graph.close();
    // Test the evolve function
    bool result = graph.evolve(&observation);
    // Check if the evolve function returns the correct result
    EXPECT_TRUE(result);
    EXPECT_EQ(graph.getState(), procedural::GraphState::Finished);
}



int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_graph");
    return RUN_ALL_TESTS();
}