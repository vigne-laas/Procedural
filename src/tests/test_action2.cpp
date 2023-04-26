#include <vector>
#include <iostream>
#include "procedural/core/Types/SpecializedAction.h"
#include "procedural/core/Types/Action.h"
#include "procedural/utils/ActionRecognition.h"
#include "procedural/utils/TimeStamp.h"
#include "ros/ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_action");

    std::vector<procedural::Action*> Actions_;
    std::cout << "============================ Create data for grasp =============================" << std::endl;


    procedural::PatternFact F(true, "A", "MoveThrought", "O", false);
    procedural::PatternFact F1(true, "A", "hasInHand", "O", true);
    std::vector<procedural::PatternTransitionFact_t> list_grasp_network;
    list_grasp_network.emplace_back(0, &F, 1);
    list_grasp_network.emplace_back(1, &F1, 2);
    list_grasp_network.emplace_back(0, &F1, 2);
    std::vector<procedural::ActionDescription_t> grasp_descriptions;
    grasp_descriptions.emplace_back("??", "isA", "graspAction");
    grasp_descriptions.emplace_back("??", "isPerformedBy", "?A");
    grasp_descriptions.emplace_back("??", "isPerformedOn", "?O");
    procedural::SpecializedAction pattern_grasp("grasp", list_grasp_network, {}, grasp_descriptions, 2, 20);

    procedural::Action grasp("grasp");
    grasp.addPatterns(pattern_grasp);

    Actions_.push_back(&grasp);
    std::cout << "============================ Create data for release =============================" << std::endl;


    procedural::PatternFact F2(true, "A", "MoveThrought", "S", false);
    procedural::PatternFact F3(false, "A", "hasInHand", "O", true);
    std::vector<procedural::PatternTransitionFact_t> release_network;
    release_network.emplace_back(0, &F2, 1);
    release_network.emplace_back(1, &F3, 2);
    release_network.emplace_back(0, &F3, 2);
    std::vector<procedural::ActionDescription_t> release_descriptions;
    release_descriptions.emplace_back("??", "isA", "release");
    release_descriptions.emplace_back("??", "isPerformedBy", "?A");
    release_descriptions.emplace_back("??", "isPerformedOn", "?O");
    release_descriptions.emplace_back("??", "isPerformedOver", "?S");
    procedural::SpecializedAction pattern_release("release", release_network, {}, release_descriptions, 2, 20);

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

    procedural::PatternFact F4 = procedural::PatternFact(false, "O", "overSupport", "S", true);
    procedural::PatternFact F5 = procedural::PatternFact(false, "O", "inContainer", "C", true);

    std::map<std::string, std::string> remap = {{"A", "A"},
                                                {"O", "O"}};


    std::vector<procedural::PatternTransitionNetwork_t> patterns_net;
    patterns_net.emplace_back(0, 1, "grasp", remap);

    std::vector<procedural::PatternTransitionFact_t> list_pick_over;
    list_pick_over.emplace_back(1, &F4, 2);
    list_pick_over.emplace_back(0, &F4, 2);

    std::vector<procedural::PatternTransitionFact_t> list_pick2_into;
    list_pick2_into.emplace_back(1, &F5, 2);
    list_pick2_into.emplace_back(0, &F5, 2);

    std::vector<procedural::SpecializedAction> list_pattern_pick;
    list_pattern_pick.emplace_back("pick_over", list_pick_over, patterns_net, over_descriptions, 2, 15);
    list_pattern_pick.emplace_back("pick_in", list_pick2_into, patterns_net, in_descriptions, 2, 15);

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

    procedural::PatternFact F6 = procedural::PatternFact(true, "O", "overSupport", "S", true);
    procedural::PatternFact F7 = procedural::PatternFact(true, "O", "inContainer", "C", true);

    std::map<std::string, std::string> remap_place = {{"A", "A"},
                                                      {"O", "O"}};


    std::vector<procedural::PatternTransitionNetwork_t> patterns_net_release;
    patterns_net_release.emplace_back(1, 2, "release", remap_place);

    std::vector<procedural::PatternTransitionFact_t> list_place_over;
//    list_pick_over.emplace_back(1, &F6, 2);
    list_place_over.emplace_back(0, &F6, 1);

    std::vector<procedural::PatternTransitionFact_t> list_place_into;
    list_place_into.emplace_back(0, &F7, 1);
//    list_pick2_into.emplace_back(0, &F7, 2);

    std::vector<procedural::SpecializedAction> list_pattern_place;
    list_pattern_place.emplace_back("place_over", list_place_over, patterns_net_release, place_over_descriptions, 1,
                                    15);
    list_pattern_place.emplace_back("place_in", list_place_into, patterns_net_release, place_in_descriptions, 1, 15);

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

    std::vector<procedural::PatternTransitionFact_t> list_pick_place;
    std::vector<procedural::SpecializedAction> list_pattern_pick_place;
    procedural::SpecializedAction pattern_pick_place("pick&place", list_pick_place, patterns_net_pick_place,
                                                     pick_place_descriptions, 2, 15);
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

    procedural::ActionRecognition recognition(Actions_);

    std::vector<procedural::Fact> facts;
//    procedural::TimeStamp_t t0(0, 50);
//    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube", 1, t0);
//    procedural::TimeStamp_t t1(15,0);
//    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube", 1,t1);

//    procedural::TimeStamp_t t2(20,0);
//    facts.emplace_back(true, "Bastien", "hasInHand", "Cube", 2,t2);
//    procedural::TimeStamp_t t3(25, 0);
//    facts.emplace_back(false, "Cube", "overSupport", "Table", 3, t3);
//    procedural::TimeStamp_t t4(5, 0);
//    facts.emplace_back(true, "Bob", "hasInHand", "Cube2", 4, t4);
//    facts.emplace_back(true, "Charly", "hasInHand", "Cube", 5);
/*** Pour tester le timeout de release  t6 t7p et 2 boucle de process***/
    procedural::TimeStamp_t t6(30, 0);
    facts.emplace_back(true, "Cube", "overSupport", "Armoire", 6, t6);
    procedural::TimeStamp_t t7p(40, 0);
    facts.emplace_back(true, "Bastien", "MoveThrought", "Armoire", 6, t7p);
//    procedural::TimeStamp_t t7(28, 0);
//    facts.emplace_back(false, "Bastien", "hasInHand", "Cube", 7, t7);
//     facts.emplace_back(false, "Cube2", "bruit", "Table",7);

    procedural::TimeStamp_t current_time(45, 0);
    for (auto& fact: facts)
    {
        recognition.addToQueue(&fact);
    }
    recognition.processQueue(current_time);
    std::cout << "\n\n\n\n" << std::endl;
    procedural::TimeStamp_t current_time2(46, 0);
    std::cout << "process time : " << current_time2 <<std::endl;

//    for (auto& action: Actions_)
//    {
////        action->checkCompleteNetworks();
//        std::cout << action->toString() << std::endl;
////        std::cout << action->checkCompleteNetworks() << std::endl;
////        action->checkCompleteNetworks();
//    }

    recognition.processQueue(current_time2);

    for (auto& action: Actions_)
        std::cout << action->toString() << std::endl;





}