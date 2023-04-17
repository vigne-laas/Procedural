#include <vector>
#include <iostream>
#include "procedural/core/Types/PatternRecognition.h"
#include "procedural/core/Types/Action.h"

#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_action");

    std::vector<procedural::Action*> Actions_;
    std::cout << "============================ Create data for grasp =============================" << std::endl;


    procedural::FactPattern F(true, "A", "MoveThrought", "O", false);
    procedural::FactPattern F1(true, "A", "hasInHand", "O", true);
    std::vector<procedural::PatternTransition_t> list_grasp_network;
    list_grasp_network.emplace_back(0, &F, 1);
    list_grasp_network.emplace_back(1, &F1, 2);
    std::vector<procedural::ActionDescription_t> grasp_descriptions;
    grasp_descriptions.emplace_back("??", "isA", "graspAction");
    grasp_descriptions.emplace_back("??", "isPerformedBy", "?A");
    grasp_descriptions.emplace_back("??", "isPerformedOn", "?O");
    procedural::PatternRecognition pattern_grasp("grasp", list_grasp_network, {}, grasp_descriptions, 20);

    procedural::Action grasp("grasp");
    grasp.addPatterns(pattern_grasp);

    Actions_.push_back(&grasp);
    std::cout << "============================ Create data for release =============================" << std::endl;


    procedural::FactPattern F2(true, "A", "MoveThrought", "O", false);
    procedural::FactPattern F3(false, "A", "hasInHand", "O", true);
    std::vector<procedural::PatternTransition_t> release_network;
    release_network.emplace_back(0, &F2, 1);
    release_network.emplace_back(1, &F3, 2);
    release_network.emplace_back(0, &F3, 2);
    std::vector<procedural::ActionDescription_t> release_descriptions;
    release_descriptions.emplace_back("??", "isA", "release");
    release_descriptions.emplace_back("??", "isPerformedBy", "?A");
    release_descriptions.emplace_back("??", "isPerformedOn", "?O");
    procedural::PatternRecognition pattern_release("release", release_network, {}, release_descriptions, 20);

    procedural::Action release("release");
    release.addPatterns(pattern_release);

    Actions_.push_back(&release);

    std::cout << "============================ Create data for pick =============================" << std::endl;


    std::vector<procedural::ActionDescription_t> over_descriptions;
    over_descriptions.emplace_back("??", "isA", "PickActionOver");
    over_descriptions.emplace_back("??", "isPerformedBy", "?A");
    over_descriptions.emplace_back("??", "isPerformedOn", "?O");
    over_descriptions.emplace_back("??", "isPerformedFrom", "?S");
    std::vector<procedural::ActionDescription_t> in_descriptions;
    in_descriptions.emplace_back("??", "isA", "PickActionInto");
    in_descriptions.emplace_back("??", "isPerformedBy", "?A");
    in_descriptions.emplace_back("??", "isPerformedOn", "?O");
    in_descriptions.emplace_back("??", "isPerformedFrom", "?C");

    procedural::FactPattern F4 = procedural::FactPattern(false, "O", "overSupport", "S", true);
    procedural::FactPattern F5 = procedural::FactPattern(false, "O", "inContainer", "C", true);

    std::map<std::string, std::string> remap = {{"A", "A"},
                                                {"O", "O"}};


    std::vector<procedural::PatternTransitionNetwork_t> patterns_net;
    patterns_net.emplace_back(0, 1, "grasp", remap);

    std::vector<procedural::PatternTransition_t> list_pick_over;
    list_pick_over.emplace_back(1, &F4, 2);
    list_pick_over.emplace_back(0, &F4, 2);

    std::vector<procedural::PatternTransition_t> list_pick2_into;
    list_pick2_into.emplace_back(1, &F5, 2);
    list_pick2_into.emplace_back(0, &F5, 2);

    std::vector<procedural::PatternRecognition> list_pattern_pick;
    list_pattern_pick.emplace_back("pick_over", list_pick_over, patterns_net, over_descriptions, 20);
    list_pattern_pick.emplace_back("pick_in", list_pick2_into, patterns_net, in_descriptions, 20);

    procedural::Action Apick("pick");
    for (auto& pattern: list_pattern_pick)
        Apick.addPatterns(pattern);

    Actions_.push_back(&Apick);
    std::cout << "============================ Create data for place =============================" << std::endl;


    std::vector<procedural::ActionDescription_t> place_over_descriptions;
    place_over_descriptions.emplace_back("??", "isA", "PlaceActionOver");
    place_over_descriptions.emplace_back("??", "isPerformedBy", "?A");
    place_over_descriptions.emplace_back("??", "isPerformedOn", "?O");
    place_over_descriptions.emplace_back("??", "isPerformedFrom", "?S");
    std::vector<procedural::ActionDescription_t> place_in_descriptions;
    place_in_descriptions.emplace_back("??", "isA", "PickActionInto");
    place_in_descriptions.emplace_back("??", "isPerformedBy", "?A");
    place_in_descriptions.emplace_back("??", "isPerformedOn", "?O");
    place_in_descriptions.emplace_back("??", "isPerformedFrom", "?C");

    procedural::FactPattern F6 = procedural::FactPattern(true, "O", "overSupport", "S", true);
    procedural::FactPattern F7 = procedural::FactPattern(true, "O", "inContainer", "C", true);

    std::map<std::string, std::string> remap_place = {{"A", "A"},
                                                {"O", "O"}};


    std::vector<procedural::PatternTransitionNetwork_t> patterns_net_release;
    patterns_net_release.emplace_back(1, 2, "release", remap_place);

    std::vector<procedural::PatternTransition_t> list_place_over;
//    list_pick_over.emplace_back(1, &F6, 2);
    list_place_over.emplace_back(0, &F6, 1);

    std::vector<procedural::PatternTransition_t> list_place_into;
    list_place_into.emplace_back(0, &F7, 1);
//    list_pick2_into.emplace_back(0, &F7, 2);

    std::vector<procedural::PatternRecognition> list_pattern_place;
    list_pattern_place.emplace_back("place_over", list_place_over, patterns_net_release, place_over_descriptions, 20);
    list_pattern_place.emplace_back("place_in", list_place_into, patterns_net_release, place_in_descriptions, 20);

    procedural::Action Aplace("place");
    for (auto& pattern: list_pattern_place)
        Aplace.addPatterns(pattern);

    Actions_.push_back(&Aplace);
    std::cout << "============================ Create data for pick&place =============================" << std::endl;


    std::vector<procedural::ActionDescription_t> pick_place_descriptions;
    pick_place_descriptions.emplace_back("??", "isA", "Pick_place_action");
    pick_place_descriptions.emplace_back("??", "isPerformedBy", "?A");
    pick_place_descriptions.emplace_back("??", "isPerformedOn", "?O");
    pick_place_descriptions.emplace_back("??", "isPerformedFrom", "?S1");
    pick_place_descriptions.emplace_back("??", "isPerformedTo", "?S2");

    std::map<std::string, std::string> remap_pick = {{"A", "A"},
                                                      {"O", "O"},
                                                      {"S", "S1"}};
    std::map<std::string, std::string> remap_place2 = {{"A", "A"},
                                                     {"O", "O"},
                                                     {"S", "S2"}};


    std::vector<procedural::PatternTransitionNetwork_t> patterns_net_pick_place;
    patterns_net_pick_place.emplace_back(0, 1, "pick_over", remap_pick);
    patterns_net_pick_place.emplace_back(1, 2, "place_over", remap_place2);

    std::vector<procedural::PatternTransition_t> list_pick_place;
    std::vector<procedural::PatternRecognition> list_pattern_pick_place;
    procedural::PatternRecognition pattern_pick_place("pick&place", list_pick_place, patterns_net_pick_place, pick_place_descriptions, 20);
//    list_pattern_pick_place.emplace_back("pick&place", list_pick_place, patterns_net_pick_place, pick_place_descriptions, 20);


    procedural::Action Apick_place("pick&place");
    Apick_place.addPatterns(pattern_pick_place);
//    for (auto& pattern: list_pattern_pick_place)
//        Apick_place.addPatterns(pattern);

    Actions_.push_back(&Apick_place);
    std::cout << "======================= Display action : ==========================================" << std::endl;
    for (auto& action: Actions_)
        std::cout << action->toString() << std::endl;
    std::cout
            << "======================= Feed actions pick with 2 networks type: =========================================="
            << std::endl;

    std::vector<procedural::Fact> facts;
    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube", 1);


    facts.emplace_back(true, "Bastien", "hasInHand", "Cube", 2);
    facts.emplace_back(false, "Cube", "overSupport", "Table", 3);

    //    facts.emplace_back(true, "Bob", "hasInHand", "Cube2", 4);
//    facts.emplace_back(true, "Charly", "hasInHand", "Cube", 5);
    facts.emplace_back(true, "Cube", "overSupport", "Armoire", 6);
    facts.emplace_back(false, "Bastien", "hasInHand", "Cube", 7);
//     facts.emplace_back(false, "Cube2", "bruit", "Table",7);

    for (auto& fact: facts)
    {
        std::cout << "--------------" << std::endl;
        std::cout << "fact : " << fact.toString() << std::endl;
        std::set<uint32_t> set_id_facts;
        std::vector<procedural::Action*> complete_actions;
        for(auto& action : Actions_)
            action->feed(&fact);

        int nb_update = 0;
        do
        {
            nb_update = 0;
            for(auto& action: Actions_)
            {
                std::set<uint32_t> temp_set = action->checkCompleteNetworks();
                if (temp_set.empty() == false)
                {
                    set_id_facts.insert(temp_set.begin(), temp_set.end());
                    complete_actions.push_back(action);
                    nb_update++;
                }

            }
//            std::cout<< "nb complete network : " << nb_update << std::endl;
//            nb_update = (int)complete_actions.size();
            for (auto& action_complete: complete_actions)
                for (auto& action: Actions_)
                    if (action != action_complete)
                        if(action->checkSubAction(action_complete))
                        {
                            std::cout << "\t\t\t update for "<<action->getName()<<"evolve thanks to complete of sub action : " << action_complete->getName() << std::endl;
                            nb_update++;
                        }
            if(nb_update!=0)
                for(auto& action: Actions_)
                    action->cleanPatterns(set_id_facts);
//            for(auto& action: Actions_)
//                action->clean();
            complete_actions.clear();
        } while (nb_update !=0);

        for (auto& action: Actions_)
            action->clean();





//        for (auto& action: Actions_)
//        {
//            action->feed(&fact);
//            std::set<uint32_t> temp_set = action->checkCompleteNetworks();
//            set_id_facts.insert(temp_set.begin(), temp_set.end());
//            if (temp_set.empty() == false)
//                complete_actions.push_back(action);
//        }
//        std::cout << "========== check sub actions ==============" << std::endl;
//        for (auto& action_complete: complete_actions)
//        {
//            for (auto& action: Actions_)
//            {
//                if (action != action_complete)
//                {
//                    action->checkSubAction(action_complete);
//                }
//            }
//        }


//        for (auto& action: Actions_)
//        {
//            std::cout << action->toString() << std::endl;
//        action->checkCompleteNetworks();
//        }
//        std::cout << "========== clean actions ==============" << std::endl;
//        for (auto& action: Actions_)
//        {
//            action->cleanPatterns(set_id_facts);
//        }
        // A.displayCurrentState();
        // A.checkCompleteNetworks();
    }
    std::cout << "\n\n\n\n" << std::endl;
    for (auto& action: Actions_)
    {
//        action->checkCompleteNetworks();
        std::cout << action->toString() << std::endl;
//        std::cout << action->checkCompleteNetworks() << std::endl;
//        action->checkCompleteNetworks();
    }
//    std::cout << Apick.toString() << std::endl;



}