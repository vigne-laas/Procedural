#include <vector>
#include <iostream>
#include "procedural/core/Types/PatternRecognition.h"
#include "procedural/graph/Action.h"

#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_action");
    std::vector<procedural::PatternRecognition> list_pattern;
    // procedural::PatternRecognition_t pattern;
    std::vector<procedural::ActionDescription_t> over_descriptions;
    over_descriptions.emplace_back("??","isA","PlaceActionOver");
    over_descriptions.emplace_back("??","isPerformedBy","?A");
    over_descriptions.emplace_back("??","isPerformedOn","?O");
    over_descriptions.emplace_back("??","isPerformedFrom","?S");
    std::vector<procedural::ActionDescription_t> in_descriptions;
    in_descriptions.emplace_back("??","isA","PlaceActionInto");
    in_descriptions.emplace_back("??","isPerformedBy","?A");
    in_descriptions.emplace_back("??","isPerformedOn","?O");
    in_descriptions.emplace_back("??","isPerformedFrom","?C");
 
    procedural::FactPattern F = procedural::FactPattern(false, "A", "MoveThrought", "O", false);
    procedural::FactPattern F1 = procedural::FactPattern(false, "A", "hasInHand", "O", false);
    procedural::FactPattern F2 = procedural::FactPattern(true, "O", "overSupport", "S", true);
    procedural::FactPattern F3 = procedural::FactPattern(true, "O", "inContainer", "C", true);

    std::vector<procedural::PatternTransition_t> list_pick1;
    list_pick1.emplace_back(0, &F, 1); 
    list_pick1.emplace_back(0, &F1, 2); 
    list_pick1.emplace_back(1, &F1, 2); 
    list_pick1.emplace_back(2, &F2, 3); 
    list_pick1.emplace_back(1, &F2, 3); 
    list_pick1.emplace_back(0, &F2, 3); 

    std::vector<procedural::PatternTransition_t> list_pick2;
    list_pick2.emplace_back(0, &F, 1); 
    list_pick2.emplace_back(0, &F1, 2); 
    list_pick2.emplace_back(1, &F1, 2); 
    list_pick2.emplace_back(2, &F3, 3); 
    list_pick2.emplace_back(1, &F3, 3); 
    list_pick2.emplace_back(0, &F3, 3); 

    list_pattern.emplace_back("pick_over", list_pick1, over_descriptions);
    list_pattern.emplace_back("pick_in", list_pick2, in_descriptions);

    procedural::Action A("pick");
    for(auto& pattern : list_pattern)
        A.addPatterns(pattern);

    std::cout << "======================= Display action : =========================================="<<std::endl;
    std::cout << A.toString() << std::endl;
    std::cout << "======================= Feed action : =========================================="<<std::endl;

    std::vector<procedural::Fact> facts;
    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube",1);
   
     facts.emplace_back(true, "Bob", "hasInHand", "Cube2",4);
    facts.emplace_back(true, "Charly", "hasInHand", "Cube",5);
    facts.emplace_back(true, "Cube", "overSupport", "Table",6);
    // facts.emplace_back(false, "Cube2", "overSupport", "Table",7);
     facts.emplace_back(true, "Bastien", "hasInHand", "Cube",2);
    facts.emplace_back(false, "Cube", "overSupport", "Table",3);

    for (auto& fact:facts)
    {
        std::cout << "--------------" << std::endl;
        std::cout << "fact : " << fact.toString() << std::endl;
        A.feed(&fact);
        // A.displayCurrentState();
        // A.checkCompleteNetworks();
    }
    A.displayCurrentState();
    A.checkCompleteNetworks();

    std::cout << A.toString() << std::endl;

    ros::shutdown();

    return 0;
}