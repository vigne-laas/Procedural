#include <iostream>
#include "procedural/reader/YamlReader.h"

int main()
{
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projects/Procedural/catkin_ws/src/Procedural/src/reader/exemple2.yaml");
    std::vector<procedural::Action*> Actions = reader.getActions();
    std::cout << "======================= Display action : ==========================================" << std::endl;
    std::cout << Actions.front()->toString() << std::endl;
//    std::cout << "======================= Feed 1 action pick with 2 networks type: =========================================="
//              << std::endl;
//
//    std::vector <procedural::Fact> facts;
//    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube", 1);
//
//    facts.emplace_back(true, "Bob", "hasInHand", "Cube2", 4);
//    facts.emplace_back(true, "Charly", "hasInHand", "Cube", 5);
//    facts.emplace_back(true, "Cube", "overSupport", "Table", 6);
//    // facts.emplace_back(false, "Cube2", "overSupport", "Table",7);
//    facts.emplace_back(true, "Bastien", "hasInHand", "Cube", 2);
//    facts.emplace_back(false, "Cube", "overSupport", "Table", 3);
//
//    for (auto& fact: facts)
//    {
//        std::cout << "--------------" << std::endl;
//        std::cout << "fact : " << fact.toString() << std::endl;
//        Actions.front()->feed(&fact);
//        // A.displayCurrentState();
//        // A.checkCompleteNetworks();
//    }
//    .displayCurrentState();
//    Apick.checkCompleteNetworks();
//
//    std::cout << Apick.toString() << std::endl;

    std::cout
            << "======================= Feed 2 actions with 2 pattern each : =========================================="
            << std::endl;

    std::vector <procedural::Fact> facts_multi;
    facts_multi.emplace_back(true, "Bastien", "hasHandMovingToward", "Cube", 1);
//    facts_multi.emplace_back(true, "Bob", "hasInHand", "Cube4",4);
//    facts_multi.emplace_back(true, "Charly", "hasInHand", "Cube7",5);
//    facts_multi.emplace_back(true, "Cube", "isOnTopOf", "Table",6);
//    facts_multi.emplace_back(false, "Cube2", "isOnTopOf", "Table",7);
    facts_multi.emplace_back(true, "Bastien", "hasInHand", "Cube", 2);
    facts_multi.emplace_back(false, "Cube", "isOnTopOf", "Table", 3);

    facts_multi.emplace_back(true, "Bastien", "hasHandMovingToward", "Box", 8);
    facts_multi.emplace_back(true, "Cube", "isInContainer", "Box", 9);
    facts_multi.emplace_back(false, "Bastien", "hasInHand", "Cube", 10);

//    facts_multi.emplace_back(false, "Cube", "isOnTopOf", "Table",11);


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
            action->feed(&fact);
            std::set<uint32_t> temp_set = action->checkCompleteNetworks();
            set_id_facts.insert(temp_set.begin(),temp_set.end());
        }
        std::cout << "========== clean actions =============="<<std::endl;
        for(auto& action : Actions)
        {
            action->cleanPatterns(set_id_facts);
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
        std::cout << action->currentState() << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
    }
    return 0;
}