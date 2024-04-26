#include <gtest/gtest.h>
#include <ros/ros.h>
#include "procedural/action_recognition/core/internal_structures/Action.h"
#include "procedural/structures/ObservationFact.h"


class ActionTest : public ::testing::Test {
public:
    procedural::ParsedFact_t fact;
    std::vector<procedural::ParsedFact_t> facts;
    procedural::ParsedSimpleAction_t simple_action;
//    std::vector<procedural::ParsedSimpleAction_t> simple_actions;
protected:

    void SetUp() override
    {
        facts.clear();
        fact.insertion = true;
        fact.level = 1;
        fact.required = true;
        fact.subject = "subject";
        fact.property = "property";
        fact.object = "object";
        facts.push_back(fact);
        // Second fact
        fact.insertion = true;
        fact.level = 2;
        fact.required = false;
        fact.subject = "subject2";
        fact.property = "property2";
        fact.object = "object2";
        facts.push_back(fact);
        // Third fact
        fact.insertion = false;
        fact.level = 3;
        fact.required = true;
        fact.subject = "subject3";
        fact.property = "property3";
        fact.object = "object3";
        facts.push_back(fact);
        simple_action.type = "test_action0";
        simple_action.facts.facts_ = facts;
    }

    void TearDown() override
    {
        // Tear down the test
    }

};

class ActionEvolveTest : public ::testing::Test {
public:
    procedural::ParsedFact_t fact;
    std::vector<procedural::ParsedFact_t> facts;
    procedural::ParsedSimpleAction_t simple_action;
protected:
protected:

    void SetUp() override
    {
        facts.clear();
        fact.insertion = true;
        fact.level = 1;
        fact.required = false;
        fact.subject = "A";
        fact.property = "pick";
        fact.object = "B";
        facts.push_back(fact);
        fact.insertion = true;
        fact.level = 2;
        fact.required = false;
        fact.subject = "A";
        fact.property = "move";
        fact.object = "C";
        facts.push_back(fact);
        fact.insertion = true;
        fact.level = 3;
        fact.required = false;
        fact.subject = "B";
        fact.property = "isIn";
        fact.object = "C";
        facts.push_back(fact);
        simple_action.type = "test_action1";
        simple_action.facts.facts_ = facts;
        facts.clear();
    }

    void TearDown() override
    {
        // Tear down the test
    }
};


TEST_F(ActionTest, buildTest)
{
    // Create an Action instance
    procedural::Action action("test_action");

    // Test the build function
    bool result = action.build(simple_action,
                               "/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/" +
                               action.getName() + ".dot");
    auto graph = action.getFactory();
    auto nodes = graph->getNodes();
    EXPECT_EQ(nodes.size(), 4);


    // Check if the build function returns the correct result
    EXPECT_TRUE(result);

    // Check if the build function returns the correct result
    EXPECT_TRUE(result);

    // Check the structure of the graph
    auto transitions0 = nodes[0]->getTransitions();
    EXPECT_EQ(transitions0.size(), 1);
    EXPECT_EQ(transitions0[0]->getTargetId(), 10);
    auto obs0 = dynamic_cast<procedural::ObservationFact*>(transitions0[0]->getObservation());
    EXPECT_EQ(obs0->getFact().getStrSubject(), facts[0].subject);
    EXPECT_EQ(obs0->getFact().getStrObject(), facts[0].object);
    EXPECT_EQ(obs0->getFact().getStrProperty(), facts[0].property);
    EXPECT_EQ(obs0->getFact().getAdd(), facts[0].insertion);

    auto transitions10 = nodes[10]->getTransitions();
    EXPECT_EQ(transitions10.size(), 2);
    std::vector<uint64_t> expected_targets10 = {20, 30};
    for (auto& transition: transitions10) {
        EXPECT_TRUE(std::find(expected_targets10.begin(), expected_targets10.end(), transition->getTargetId()) !=
                    expected_targets10.end());
        auto obs = dynamic_cast<procedural::ObservationFact*>(transition->getObservation());
        if (transition->getTargetId() == 20) {
            EXPECT_EQ(obs->getFact().getStrSubject(), facts[1].subject);
            EXPECT_EQ(obs->getFact().getStrObject(), facts[1].object);
            EXPECT_EQ(obs->getFact().getStrProperty(), facts[1].property);
            EXPECT_EQ(obs->getFact().getAdd(), facts[1].insertion);
        } else if (transition->getTargetId() == 30) {
            EXPECT_EQ(obs->getFact().getStrSubject(), facts[2].subject);
            EXPECT_EQ(obs->getFact().getStrObject(), facts[2].object);
            EXPECT_EQ(obs->getFact().getStrProperty(), facts[2].property);
            EXPECT_EQ(obs->getFact().getAdd(), facts[2].insertion);
        }
    }

    auto transitions20 = nodes[20]->getTransitions();
    EXPECT_EQ(transitions20.size(), 1);
    EXPECT_EQ(transitions20[0]->getTargetId(), 30);
    auto obs20 = dynamic_cast<procedural::ObservationFact*>(transitions20[0]->getObservation());
    EXPECT_EQ(obs20->getFact().getStrSubject(), facts[2].subject);
    EXPECT_EQ(obs20->getFact().getStrObject(), facts[2].object);
    EXPECT_EQ(obs20->getFact().getStrProperty(), facts[2].property);
    EXPECT_EQ(obs20->getFact().getAdd(), facts[2].insertion);
}

TEST_F(ActionEvolveTest, evolveAndPropagateTest)
{
    // Create an Action instance
    procedural::Action action("test_action1");

    // Test the build function
    bool result = action.build(simple_action,
                               "/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/" +
                               action.getName() + ".dot");
    auto graph = action.getFactory();
    auto nodes = graph->getNodes();
    EXPECT_EQ(nodes.size(), 4);

    // Check if the build function returns the correct result
    EXPECT_TRUE(result);

    // Check if the build function returns the correct result
    EXPECT_TRUE(result);

    // Check the structure of the graph
    auto transitions0 = nodes[0]->getTransitions();
    EXPECT_EQ(transitions0.size(), 3);
    auto transitions10 = nodes[10]->getTransitions();
    EXPECT_EQ(transitions10.size(), 2);
    auto transitions20 = nodes[20]->getTransitions();
    EXPECT_EQ(transitions20.size(), 1);
    auto transitions30 = nodes[30]->getTransitions();
    EXPECT_EQ(transitions30.size(), 0);

    // Check the labels of the nodes
    EXPECT_EQ(nodes[0]->toString(), "test_action1_0");
    EXPECT_EQ(nodes[10]->toString(), "test_action1_10");
    EXPECT_EQ(nodes[20]->toString(), "test_action1_20");
    EXPECT_EQ(nodes[30]->toString(), "test_action1_30");

    // Check the labels of the transitions
    // Check the labels of the transitions
    auto obs0 = dynamic_cast<procedural::ObservationFact*>(transitions0[0]->getObservation());
    EXPECT_EQ(obs0->getFact().getStrSubject(), "A");
    EXPECT_EQ(obs0->getFact().getStrProperty(), "pick");
    EXPECT_EQ(obs0->getFact().getStrObject(), "B");

    obs0 = dynamic_cast<procedural::ObservationFact*>(transitions0[1]->getObservation());
    EXPECT_EQ(obs0->getFact().getStrSubject(), "A");
    EXPECT_EQ(obs0->getFact().getStrProperty(), "move");
    EXPECT_EQ(obs0->getFact().getStrObject(), "C");

    obs0 = dynamic_cast<procedural::ObservationFact*>(transitions0[2]->getObservation());
    EXPECT_EQ(obs0->getFact().getStrSubject(), "B");
    EXPECT_EQ(obs0->getFact().getStrProperty(), "isIn");
    EXPECT_EQ(obs0->getFact().getStrObject(), "C");

    auto obs10 = dynamic_cast<procedural::ObservationFact*>(transitions10[0]->getObservation());
    EXPECT_EQ(obs10->getFact().getStrSubject(), "A");
    EXPECT_EQ(obs10->getFact().getStrProperty(), "move");
    EXPECT_EQ(obs10->getFact().getStrObject(), "C");

    obs10 = dynamic_cast<procedural::ObservationFact*>(transitions10[1]->getObservation());
    EXPECT_EQ(obs10->getFact().getStrSubject(), "B");
    EXPECT_EQ(obs10->getFact().getStrProperty(), "isIn");
    EXPECT_EQ(obs10->getFact().getStrObject(), "C");

    auto obs20 = dynamic_cast<procedural::ObservationFact*>(transitions20[0]->getObservation());
    EXPECT_EQ(obs20->getFact().getStrSubject(), "B");
    EXPECT_EQ(obs20->getFact().getStrProperty(), "isIn");
    EXPECT_EQ(obs20->getFact().getStrObject(), "C");


    // Test the evolve function
    auto timestamp = procedural::TimeStamp_t();
    auto test_fact = procedural::Fact(true, "agent", "pick", "bowl", timestamp);
    procedural::ObservationFact observation(test_fact);
    LOG_DEBUG << "Observation : " << observation.toString();
    LOG_DEBUG << "Observation variables : " << observation.table_variables_.toString();
    auto res = action.evolve(&observation);
    graph = action.getActiveGraphs().back();
    nodes = graph->getNodes();
    EXPECT_TRUE(res);
    // Check the structure of the graph
    LOG_DEBUG << "Graph after first evolution";
    LOG_DEBUG << graph->getTableVariables().toString();
    for (const auto& nodePair: nodes) {
        auto transitions = nodePair.second->getTransitions();
//        LOG_DEBUG << "Node " << nodePair.first << " has " << transitions.size() << " transitions";
//        LOG_DEBUG << nodePair.second->toString();
//        LOG_DEBUG << "Transitions:";
        for (const auto& transition: transitions) {
            auto obs = dynamic_cast<procedural::ObservationFact*>(transition->getObservation());
//            LOG_DEBUG << *obs << "variables : " << obs->table_variables_.toString();
            if (obs) {
                auto fact = obs->getFact();
//                LOG_DEBUG << "Fact to check : " << fact.toString();
                if (fact.getLiteralSubject() == "A") {
//                    LOG_DEBUG << "Subject is agent ?= " << fact.getStrSubject();
                    EXPECT_EQ(fact.getStrSubject(), "agent");
                } else if (fact.getLiteralSubject() == "B") {
                    EXPECT_EQ(fact.getStrSubject(), "bowl");
                } else if (fact.getLiteralObject() == "B") {
                    EXPECT_EQ(fact.getStrObject(), "bowl");
                } else if (fact.getLiteralObject() == "A") {
                    EXPECT_EQ(fact.getStrObject(), "agent");
                }
            }
        }
    }
    graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/" +
                   action.getName() + "_evolve1.dot");

    auto factory = action.getFactory();
    EXPECT_EQ(factory->getState(), procedural::GraphState::Closed);
    EXPECT_EQ(factory->getInitialNode(),factory->getCurrentNode());
    EXPECT_EQ(factory->getCompletionRatio(),0.0);
    EXPECT_EQ(factory->getAdvancementRatio(),0.0);

    // Test with a second observation
    test_fact = procedural::Fact(true, "agent", "move", "table", timestamp);
    procedural::ObservationFact observation2(test_fact);
    LOG_DEBUG << "Observation 2 : " << observation2.toString();
    LOG_INFO << "Trying to evolve with observation 2";
    res = action.evolve(&observation2);
    EXPECT_TRUE(res);
    nodes = graph->getNodes();
    LOG_DEBUG << "Graph after second evolution";
    LOG_DEBUG << graph->getTableVariables().toString();
    for (const auto& nodePair: nodes) {
        auto transitions = nodePair.second->getTransitions();
        for (const auto& transition: transitions) {
            auto obs = dynamic_cast<procedural::ObservationFact*>(transition->getObservation());
            if (obs) {
                auto fact = obs->getFact();
//                LOG_DEBUG << "Fact to check : " << fact.toString();
                if (fact.getLiteralSubject() == "A") {
                    EXPECT_EQ(fact.getStrSubject(), "agent");
                } else if (fact.getLiteralSubject() == "B") {
                    EXPECT_EQ(fact.getStrSubject(), "bowl");
                } else if (fact.getLiteralObject() == "B") {
                    EXPECT_EQ(fact.getStrObject(), "bowl");
                } else if (fact.getLiteralObject() == "A") {
                    EXPECT_EQ(fact.getStrObject(), "agent");
                } else if (fact.getLiteralObject() == "C") {
                    EXPECT_EQ(fact.getStrObject(), "table");
                } else if (fact.getLiteralSubject() == "C") {
                    EXPECT_EQ(fact.getStrSubject(), "table");
                }

            }
        }
    }
    graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/" +
                   action.getName() + "_evolve2.dot");
    // Test with a third observation
    test_fact = procedural::Fact(false, "bowl", "isIn", "table", timestamp);
    procedural::ObservationFact observation3(test_fact);
    LOG_DEBUG << "Observation 3 : " << observation3.toString();
    LOG_INFO << "Trying to evolve with observation 3";
    res = action.evolve(&observation3);
    EXPECT_FALSE(res);
    test_fact = procedural::Fact(true, "bowl", "isIn", "table", timestamp);
    procedural::ObservationFact observation4(test_fact);
    LOG_DEBUG << "Observation 4 : " << observation4.toString();
    LOG_INFO << "Trying to evolve with observation 4";
    res = action.evolve(&observation4);
    EXPECT_TRUE(res);
    graph->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/" +
                   action.getName() + "_evolve3.dot");
//    EXPECT_EQ(graph->getState(), procedural::GraphState::Completed);
    EXPECT_EQ(graph->getCompletionRatio(), 1.0);
    EXPECT_EQ(graph->getAdvancementRatio(), 1.0);
    EXPECT_TRUE(action.getActiveGraphs().empty());
    EXPECT_EQ(action.getFinishedGraphs().size(), 1);
    auto finished_graph = action.getFinishedGraphs().back();

    EXPECT_EQ(finished_graph->getState(), procedural::GraphState::Completed);
    factory = action.getFactory();
    EXPECT_EQ(factory->getState(), procedural::GraphState::Closed);
    EXPECT_EQ(factory->getInitialNode(),factory->getCurrentNode());
    EXPECT_EQ(factory->getCompletionRatio(),0.0);
    EXPECT_EQ(factory->getAdvancementRatio(),0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_action");
    return RUN_ALL_TESTS();
}