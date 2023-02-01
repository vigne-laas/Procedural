#include <vector>
#include <iostream>
#include "procedural/core/Types/PatternRecognition.h"
#include "procedural/graph/Action.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_action");
    std::vector<procedural::PatternRecognition_t> list_pattern;
    procedural::PatternRecognition_t pattern;
    procedural::FactPattern F = procedural::FactPattern(false, "A", "MoveThrought", "O", false);
    procedural::FactPattern F1 = procedural::FactPattern(false, "A", "hasInHand", "O", false);
    procedural::FactPattern F2 = procedural::FactPattern(true, "O", "overSupport", "S", true);
    procedural::FactPattern F3 = procedural::FactPattern(true, "O", "inContainer", "S", true);
    pattern.patterns.resize(3);
    pattern.patterns[0].push_back(F);
    pattern.patterns[1].push_back(F1);
    pattern.patterns[2].push_back(F2);
    pattern.patterns[2].push_back(F3);

    procedural::Action A("pick");
    A.addPatterns(pattern);

    std::vector<procedural::Fact> facts;
    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube");
   facts.emplace_back(true, "Bob", "hasInHand", "Cube2");
   facts.emplace_back(true, "Cube", "hasInHand", "Cube");
    facts.emplace_back(true, "Bastien", "hasInHand", "Cube");
   facts.emplace_back(true, "Cube", "overSupport", "Table");
    facts.emplace_back(false, "Cube", "overSupport", "Table");
    facts.emplace_back(false, "Cube2", "overSupport", "Table");
    for (auto& fact:facts)
    {
        std::cout << "--------------" << std::endl;
        std::cout << "fact : " << fact.toString() << std::endl;
        A.feed(fact);
        A.displayCurrentState();

    }

    ros::shutdown();

    return 0;
}