#include <iostream>
#include "procedural/reader/YamlReader.h"
#include "procedural/reader/ActionBuilder.h"
#include "procedural/core/ActionRecognition.h"

int main()
{
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/reader/test.yaml");
    std::cout << "simple action : " << std::endl;
    for (auto& action: reader.getSimpleActions())
        std::cout << action << std::endl;
    std::cout << "composed action : " << std::endl;
    for (auto& action: reader.getComposedActions())
        std::cout << action << std::endl;

    procedural::ActionBuilder builder(reader.getSimpleActions(), reader.getComposedActions());
    auto Actions_ = builder.getActions();
    procedural::ActionRecognition recognition;
    recognition.init(Actions_);


    std::vector<procedural::Fact> facts;
//    procedural::TimeStamp_t t0(0, 50);
//    facts.emplace_back(true, "Bastien", "hasHandMovingToward", "Cube", 1, t0);
    procedural::TimeStamp_t t1(15, 0);
    facts.emplace_back(true, "Bastien", "hasHandMovingToward", "Cube", 1, t1);

    procedural::TimeStamp_t t2(20, 0);
    facts.emplace_back(true, "Bastien", "hasInHand", "Cube", 2, t2);
    procedural::TimeStamp_t t3(25, 0);
    facts.emplace_back(false, "Cube", "isOnTopOf", "Table", 3, t3);
//    procedural::TimeStamp_t t4(5, 0);
//    facts.emplace_back(true, "Bob", "hasInHand", "Cube2", 4, t4);
/*** Pour tester le timeout de release  t6 = 30 t7p=40 et 2 boucle de process ***/
    procedural::TimeStamp_t t6(27, 0);
    facts.emplace_back(true, "Cube", "isOnTopOf", "Armoire", 6, t6);
//    procedural::TimeStamp_t t7p(40, 0);
//    facts.emplace_back(true, "Bastien", "MoveThrought", "Armoire", 6, t7p);
    procedural::TimeStamp_t t7(28, 0);
    facts.emplace_back(false, "Bastien", "hasInHand", "Cube", 7, t7);

    procedural::TimeStamp_t current_time(30, 0);
    for (auto& fact: facts)
    {
        recognition.addToQueue(&fact);
    }
    recognition.processQueue(current_time);
    std::cout << "\n\n\n\n" << std::endl;
//    procedural::TimeStamp_t current_time2(46, 0);
//    std::cout << "process time : " << current_time2 <<std::endl;

    for (auto& action: Actions_)
        std::cout << action->toString() << std::endl;

//    recognition.processQueue(current_time2);
//
//    for (auto& action: Actions_)
//        std::cout << action->toString() << std::endl;



    return 0;
}