#include <vector>
#include <iostream>
#include "procedural/core/Types/SpecializedAction.h"
#include "procedural/core/Types/Action.h"

#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_action");

    // procedural::PatternRecognition_t pattern;
    std::cout << "============================ Create data for pick =============================" << std::endl;
    std::vector <procedural::ActionDescription_t> over_descriptions;
    over_descriptions.emplace_back("??", "isA", "PickActionOver");
    over_descriptions.emplace_back("??", "isPerformedBy", "?A");
    over_descriptions.emplace_back("??", "isPerformedOn", "?O");
    over_descriptions.emplace_back("??", "isPerformedFrom", "?S");
    std::vector <procedural::ActionDescription_t> in_descriptions;
    in_descriptions.emplace_back("??", "isA", "PickActionInto");
    in_descriptions.emplace_back("??", "isPerformedBy", "?A");
    in_descriptions.emplace_back("??", "isPerformedOn", "?O");
    in_descriptions.emplace_back("??", "isPerformedFrom", "?C");

    procedural::PatternFact F = procedural::PatternFact(true, "A", "MoveThrought", "O", false);
    procedural::PatternFact F1 = procedural::PatternFact(true, "A", "hasInHand", "O", false);
    procedural::PatternFact F2 = procedural::PatternFact(false, "O", "overSupport", "S", true);
    procedural::PatternFact F3 = procedural::PatternFact(false, "O", "inContainer", "C", true);

    std::vector <procedural::PatternTransitionFact_t> list_pick1;
    list_pick1.emplace_back(0, &F, 1);
    list_pick1.emplace_back(0, &F1, 2);
    list_pick1.emplace_back(1, &F1, 2);
    list_pick1.emplace_back(2, &F2, 3);
    list_pick1.emplace_back(1, &F2, 3);
    list_pick1.emplace_back(0, &F2, 3);

    std::vector <procedural::PatternTransitionFact_t> list_pick2;
    list_pick2.emplace_back(0, &F, 1);
    list_pick2.emplace_back(0, &F1, 2);
    list_pick2.emplace_back(1, &F1, 2);
    list_pick2.emplace_back(2, &F3, 3);
    list_pick2.emplace_back(1, &F3, 3);
    list_pick2.emplace_back(0, &F3, 3);

    std::vector <procedural::PatternRecognition> list_pattern_pick;
    list_pattern_pick.emplace_back("pick_over", list_pick1, over_descriptions,4);
    list_pattern_pick.emplace_back("pick_in", list_pick2, in_descriptions,4);

    std::cout << "============================ Create data for place =============================" << std::endl;
    std::vector <procedural::ActionDescription_t> over_place_descriptions;
    over_place_descriptions.emplace_back("??", "isA", "PlaceActionOver");
    over_place_descriptions.emplace_back("??", "isPerformedBy", "?A");
    over_place_descriptions.emplace_back("??", "isPerformedOn", "?O");
    over_place_descriptions.emplace_back("??", "isPerformedFrom", "?S");
    std::vector <procedural::ActionDescription_t> in_place_descriptions;
    in_place_descriptions.emplace_back("??", "isA", "PlaceActionInto");
    in_place_descriptions.emplace_back("??", "isPerformedBy", "?A");
    in_place_descriptions.emplace_back("??", "isPerformedOn", "?O");
    in_place_descriptions.emplace_back("??", "isPerformedFrom", "?S");

    procedural::PatternFact Fp = procedural::PatternFact(true, "A", "MoveThrought", "S", false);
    procedural::PatternFact Fp1 = procedural::PatternFact(true, "O", "isInContainer", "S", true);
    procedural::PatternFact Fp2 = procedural::PatternFact(true, "O", "isOnTopOf", "S", true);
    procedural::PatternFact Fp3 = procedural::PatternFact(false, "A", "hasInHand", "O", true);

    std::vector <procedural::PatternTransitionFact_t> list_place1;
    list_place1.emplace_back(0, &Fp, 1);
    list_place1.emplace_back(0, &Fp1, 2);
    list_place1.emplace_back(1, &Fp1, 2);
    list_place1.emplace_back(2, &Fp3, 3);

    std::vector <procedural::PatternTransitionFact_t> list_place2;
    list_place2.emplace_back(0, &Fp, 1);
    list_place2.emplace_back(0, &Fp2, 2);
    list_place2.emplace_back(1, &Fp2, 2);
    list_place2.emplace_back(2, &Fp3, 3);

    std::vector <procedural::PatternRecognition> list_pattern_place;
    list_pattern_place.emplace_back("place_over", list_place2, over_place_descriptions,4);
    list_pattern_place.emplace_back("place_in", list_place1, in_place_descriptions,4);

    std::cout << "====================================== Create 2 actions ================================"
              << std::endl;


    procedural::Action Apick("pick");
    procedural::Action Aplace("place");
    for (auto& pattern: list_pattern_pick)
        Apick.addPatterns(pattern);
    for (auto& pattern: list_pattern_place)
        Aplace.addPatterns(pattern);

    std::vector <procedural::Action> Actions;
    Actions.push_back(Apick);
    Actions.push_back(Aplace);


    std::cout << "======================= Display action : ==========================================" << std::endl;
    std::cout << Apick.toString() << std::endl;
    std::cout << "======================= Feed 1 action pick with 2 networks type: =========================================="
            << std::endl;

    std::vector <procedural::Fact> facts;
    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube", 1);

    facts.emplace_back(true, "Bob", "hasInHand", "Cube2", 4);
    facts.emplace_back(true, "Charly", "hasInHand", "Cube", 5);
    facts.emplace_back(true, "Cube", "overSupport", "Table", 6);
    // facts.emplace_back(false, "Cube2", "overSupport", "Table",7);
    facts.emplace_back(true, "Bastien", "hasInHand", "Cube", 2);
    facts.emplace_back(false, "Cube", "overSupport", "Table", 3);

    for (auto& fact: facts)
    {
        std::cout << "--------------" << std::endl;
        std::cout << "fact : " << fact.toString() << std::endl;
        Apick.feed(&fact);
        // A.displayCurrentState();
        // A.checkCompleteNetworks();
    }
    Apick.displayCurrentState();
    Apick.checkCompleteNetworks();

    std::cout << Apick.toString() << std::endl;

    std::cout
            << "======================= Feed 2 actions with 2 pattern each : =========================================="
            << std::endl;

    std::vector <procedural::Fact> facts_multi;
    facts_multi.emplace_back(true, "Bastien", "MoveThrought", "Cube", 1);
     facts_multi.emplace_back(true, "Bob", "hasInHand", "Cube4",4);
     facts_multi.emplace_back(true, "Charly", "hasInHand", "Cube7",5);
     facts_multi.emplace_back(true, "Cube", "overSupport", "Table",6);
     facts_multi.emplace_back(false, "Cube2", "overSupport", "Table",7);
    facts_multi.emplace_back(true, "Bastien", "hasInHand", "Cube", 2);
    facts_multi.emplace_back(false, "Cube", "overSupport", "Table", 3);

    facts_multi.emplace_back(true, "Bastien", "MoveThrought", "Box", 8);
    facts_multi.emplace_back(true, "Cube", "isInContainer", "Box", 9);
    facts_multi.emplace_back(false, "Bastien", "hasInHand", "Cube", 10);

     facts_multi.emplace_back(false, "Cube", "overSupport", "Table",11);


    for (auto& fact: facts_multi)
    {
        std::cout << "--------------------------------- new fact ------------------------------" << std::endl;
        std::cout << "fact : " << fact.toString() << std::endl;
//        Aplace.feed(&fact);
//        std::cout << Aplace.toString() << std::endl;
//        Aplace.checkCompleteNetworks();
        std::set <uint32_t> set_id_facts;
        for (auto& action: Actions)
        {
            action.feed(&fact);
            std::set<uint32_t> temp_set = action.checkCompleteNetworks();
            set_id_facts.insert(temp_set.begin(),temp_set.end());
        }
        std::cout << "========== clean actions =============="<<std::endl;
        for(auto& action : Actions)
        {
            action.cleanPatterns(set_id_facts);
        }

        // A.displayCurrentState();
        // A.checkCompleteNetworks();
    }
    std::cout << "====================== Final state of each action : ======================================"
              << std::endl;
    for (auto& action: Actions)
    {
//         action.displayCurrentState();

//         std::cout << action.toString() << std::endl;
        std::cout << action.currentState() << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
    }


    ros::shutdown();

    return 0;
}