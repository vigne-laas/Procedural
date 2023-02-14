#include <vector>
#include <iostream>
#include "procedural/core/Types/PatternRecognition.h"
#include "procedural/graph/Action.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_action");
    std::vector<procedural::PatternRecognition_t> list_pattern;
    // procedural::PatternRecognition_t pattern;
    std::vector<std::string> description1 {"hello pick1","pick1","1"};
    std::vector<std::string> description2 {"hello pick2","pick2","2"};
    procedural::FactPattern F = procedural::FactPattern(false, "A", "MoveThrought", "O", false);
    procedural::FactPattern F1 = procedural::FactPattern(false, "A", "hasInHand", "O", false);
    procedural::FactPattern F2 = procedural::FactPattern(true, "O", "overSupport", "S", true);
    procedural::FactPattern F3 = procedural::FactPattern(true, "O", "inContainer", "S", true);

    std::vector<procedural::PatternTransition_t> list_pick1;
    list_pick1.emplace_back(0, &F, 1); 
    list_pick1.emplace_back(0, &F1, 2); 
    list_pick1.emplace_back(1, &F1, 2); 
    list_pick1.emplace_back(2, &F2, 3); 
    list_pick1.emplace_back(1, &F2, 3); 
    list_pick1.emplace_back(0, &F2, 3); 

    std::vector<procedural::PatternTransition_t> list_pick2;
    list_pick1.emplace_back(0, &F, 1); 
    list_pick1.emplace_back(0, &F1, 2); 
    list_pick1.emplace_back(1, &F1, 2); 
    list_pick1.emplace_back(2, &F3, 3); 
    list_pick1.emplace_back(1, &F3, 3); 
    list_pick1.emplace_back(0, &F3, 3); 

    list_pattern.emplace_back("pick1",list_pick1,description1);
    list_pattern.emplace_back("pick2",list_pick2,description2);

    procedural::Action A("pick");
    for(auto& pattern : list_pattern)
        A.addPatterns(pattern);

    A.close();

    // std::vector<procedural::Fact> facts;
    // facts.emplace_back(true, "Bastien", "MoveThrought", "Cube");
    // facts.emplace_back(true, "Bob", "hasInHand", "Cube2");
    // facts.emplace_back(true, "Cube", "hasInHand", "Cube");
    // facts.emplace_back(true, "Bastien", "hasInHand", "Cube");
    // facts.emplace_back(true, "Cube", "overSupport", "Table");
    // facts.emplace_back(false, "Cube", "overSupport", "Table");
    // facts.emplace_back(false, "Cube2", "overSupport", "Table");
    // for (auto& fact:facts)
    // {
    //     std::cout << "--------------" << std::endl;
    //     std::cout << "fact : " << fact.toString() << std::endl;
    //     A.feed(fact);
    //     A.displayCurrentState();
    // }

    ros::shutdown();

    return 0;
}