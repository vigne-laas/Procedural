#include "procedural/old/reader/DomainReader.h"
#include "procedural/old/builder/HTNBuilder.h"
#include "procedural/old/reader/YamlReader.h"
#include "procedural/old/builder/ActionBuilder.h"
#include "procedural/old/core/Types/Task.h"
#include "procedural/old/core/TaskRecognition.h"


int main(int, const char**)
{
    procedural::DomainReader domainReader;
    procedural::HTNBuilder builder;
    domainReader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/tests/kitchen_domain/task.dom");
    domainReader.getMethods();
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/tests/kitchen_domain/actions.yaml");
    procedural::ActionBuilder action_builder(reader.getSimpleActions(), reader.getComposedActions());
    auto actions_ = action_builder.getActions();
    std::cout << std::endl;
//    std::cout << "Action Method : \n" << procedural::ActionMethod::action_method_types.toString() << std::endl;
//    std::cout << "Task : \n" << procedural::Task::task_types.toString() << std::endl;
//    std::cout << "Action : \n" << procedural::Action::action_types.toString() << std::endl;
    builder.buildTask(domainReader.getMethods(), domainReader.getActions(), action_builder.getActions(),"/home/avigne/Projets/Procedural/catkin_ws/src/Procedural");

//    builder.displayTask();

    procedural::TaskRecognition task_recognition;
    task_recognition.init(builder.getTask());
    procedural::ActionRecognition action_recognition;
    action_recognition.init(action_builder.getActions());
//    std::cout << "Task : \n" << procedural::Task::task_types.toString() << std::endl;
//    std::cout << "Action : \n" << procedural::Action::action_types.toString() << std::endl;
    action_recognition.linkToTaskRecognition(&task_recognition);

//
//    std::vector<procedural::Fact> facts;
////    procedural::TimeStamp_t t0(0, 50);
////    facts.emplace_back(true, "Bastien", "hasHandMovingToward", "Cube", 1, t0);
//    procedural::TimeStamp_t t1(15, 0);
//    facts.emplace_back(true, "Bastien", "hasHandMovingToward", "Cube", 1, t1);
//
//    procedural::TimeStamp_t t2(20, 0);
//    facts.emplace_back(true, "Bastien", "isHolding", "Cube", 2, t2);
//    procedural::TimeStamp_t t3(25, 0);
//    facts.emplace_back(false, "Cube", "isOnTopOf", "Table", 3, t3);
////    procedural::TimeStamp_t t4(5, 0);
////    facts.emplace_back(true, "Bob", "hasInHand", "Cube2", 4, t4);
///*** Pour tester le timeout de release  t6 = 30 t7p=40 et 2 boucle de process ***/
//    procedural::TimeStamp_t t6(29, 0);
//    facts.emplace_back(true, "Cube", "isOnTopOf", "Armoire", 6, t6);
////    procedural::TimeStamp_t t7p(40, 0);
////    facts.emplace_back(true, "Bastien", "MoveThrought", "Armoire", 6, t7p);
//    procedural::TimeStamp_t t7(28, 0);
//    facts.emplace_back(false, "Bastien", "isHolding", "Cube", 7, t7);
//    std::cout << std::endl << std::endl;
//    procedural::TimeStamp_t current_time(28, 500);
//    for (auto& fact: facts)
//    {
//        action_recognition.addToQueue(&fact);
//    }
//    std::cout << "------------------------- Start process --------------------------------------" << std::endl;
//    action_recognition.processQueue(current_time);
//    std::cout << "\n\n\n\n" << std::endl;
//    std::cout << "----------------------------------- Process Task ------------------------------------" << std::endl;
//    procedural::TimeStamp_t current_time2(29, 0);
//    task_recognition.process(current_time2);
//
////    procedural::TimeStamp_t current_time2(46, 0);
////    std::cout << "process time : " << current_time2 <<std::endl;
//    std::cout << "\n\n\n\n" << std::endl;
//    std::cout << "----------------------------------- Display actions  ------------------------------------" << std::endl;
//
//    for (auto& action: actions_)
//        std::cout << action->toString() << std::endl;


    return 0;
}