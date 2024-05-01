#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include "procedural/action_recognition/reader/YamlReader.h"
#include "procedural/action_recognition/core/ActionRecognition.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/utils/Logger.h"
#include "procedural/utils/WordTable.h"
#include "procedural/action_recognition/builder/ActionBuilder.h"

namespace procedural {
class TestActionRecognition : public ::testing::Test {
public:
    ActionRecognition actionRecognition;
    YamlReader yamlReader;
    ActionBuilder actionBuilder;
    static std::vector<Graph*> graphs_;
    static std::vector<Observation> observations_;


    // You can do set-up work for each test here.
    TestActionRecognition()
    {
        std::string package_path = ros::package::getPath("procedural");
        std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
        LOG_INFO << "File path: " << file_path;
        yamlReader.read(file_path);
        actionBuilder.build(yamlReader.getSimpleActions(), yamlReader.getComposedActions());

    }

    // You can do clean-up work that doesn't throw exceptions here.
    virtual ~TestActionRecognition() {}

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:
    virtual void SetUp()
    {
        actionRecognition.init(actionBuilder.getActions());
        actionRecognition.setCallback(TestActionRecognition::callbackGraph);
        actionRecognition.linkToTaskRecognition(TestActionRecognition::callbackObservation);
    }

    virtual void TearDown()
    {
        actionRecognition = ActionRecognition();
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    static void callbackGraph(const std::vector<Graph*>& graphs)
    {
//        LOG_INFO << "Callback Graph";
//        for (auto graph: graphs) {
//            LOG_INFO << "Graph: " << graph->toString();
//        }
        graphs_.insert(graphs_.end(), graphs.begin(), graphs.end());
    }

    static void callbackObservation(const std::vector<Observation>& observations)
    {
//        LOG_INFO << "Callback Observation";
//        for (auto observation: observations) {
//            LOG_INFO << "Observation: " << observation.toString();
//        }
        observations_.insert(observations_.end(), observations.begin(), observations.end());
    }


};

std::vector<Graph*> TestActionRecognition::graphs_;
std::vector<Observation> TestActionRecognition::observations_;


TEST_F(TestActionRecognition, testAddToQueue)
{
    // Test the addToQueue method
    // You will need to replace this with your actual test
    LOG_INFO << "Test Add to Queue";
    auto fact = new Fact(true, "Bob", "Agent", "hasHandMovingToward", "bowl_0", "Pickable", TimeStamp_t(0, 0));
    auto fact1 = new Fact(true, "Bob", "Agent", "isHolding", "bowl_0", "Pickable", TimeStamp_t(0, 0));
    actionRecognition.addToQueue(fact);
    actionRecognition.addToQueue(fact1);
    actionRecognition.processQueue(TimeStamp_t(0, 0));
    ASSERT_EQ(graphs_.size(), 1);
    auto graph = graphs_.front();
    ASSERT_EQ(graph->getName(), "Grasp");
    ASSERT_EQ(graph->getCompletionRatio(), 1);
    ASSERT_EQ(graph->getState(), GraphState::Completed);
    ASSERT_EQ(observations_.size(), 1);
    auto observation = observations_.front();
    ASSERT_EQ(observation.getId(),1);
    for(const auto & var : observation.table_variables_.variables)
    {
        if(var.first == "A")
        {
            ASSERT_EQ(var.second->getValue(), 1);
            ASSERT_EQ(var.second->getType(), "Agent");
        }
        if(var.first == "C")
        {
            ASSERT_EQ(var.second->getValue(), 2);
            ASSERT_EQ(var.second->getType(), "Pickable");
        }
        if(var.first == "self")
        {
            ASSERT_EQ(var.second->getType(), "Action_Grasp");
        }

    }
    graphs_.clear();
    observations_.clear();
    auto fact2 = new Fact(false, "bowl_0", "Pickable", "isIn", "box_1", "Container", TimeStamp_t(1, 0));
    actionRecognition.addToQueue(fact2);
    actionRecognition.processQueue(TimeStamp_t(1, 0));
    ASSERT_EQ(graphs_.size(), 1);
    graph = graphs_.front();
    ASSERT_EQ(graph->getName(), "Pick_In");
    ASSERT_EQ(graph->getCompletionRatio(), 1);
    ASSERT_EQ(graph->getState(), GraphState::Completed);
    ASSERT_EQ(observations_.size(), 1);
    observation = observations_.front();
    ASSERT_EQ(observation.getId(),2);
    for(const auto & var : observation.table_variables_.variables)
    {
        if(var.first == "A")
        {
            ASSERT_EQ(var.second->getValue(), 1);
            ASSERT_EQ(var.second->getType(), "Agent");
        }
        if(var.first == "O")
        {
            ASSERT_EQ(var.second->getValue(), 2);
            ASSERT_EQ(var.second->getType(), "Pickable");
        }
        if(var.first == "C")
        {
            ASSERT_EQ(var.second->getValue(), 3);
            ASSERT_EQ(var.second->getType(), "Container");
        }
        if(var.first == "self")
        {
            ASSERT_EQ(var.second->getType(), "Action_Pick_In");
        }

    }
}

//TEST_F(TestActionRecognition, testProcessQueue)
//{
//    // Test the processQueue method
//    // You will need to replace this with your actual test
//    ASSERT_TRUE(true);
//}
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_action_recognition");
    return RUN_ALL_TESTS();
}