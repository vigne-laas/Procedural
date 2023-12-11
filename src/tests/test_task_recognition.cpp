#include "procedural/reader/DomainReader.h"
#include "procedural/builder/HTNBuilder.h"
#include "procedural/reader/YamlReader.h"
#include "procedural/builder/ActionBuilder.h"
#include "procedural/core/Types/Task.h"
#include "procedural/core/TaskRecognition.h"


int main(int, const char**)
{
    procedural::DomainReader domainReader;
    procedural::HTNBuilder builder;
    domainReader.getMethods();
    procedural::YamlReader reader = procedural::YamlReader();
    reader.read("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/reader/test_link_domain.yaml");
    procedural::ActionBuilder action_builder(reader.getSimpleActions(), reader.getComposedActions());
    auto actions_ = action_builder.getActions();
    std::cout << std::endl;
//    std::cout << "Action Method : \n" << procedural::ActionMethod::action_method_types.toString() << std::endl;
    std::cout << "Task : \n" << procedural::Task::task_types.toString() << std::endl;
    std::cout << "Action : \n" << procedural::Action::action_types.toString() << std::endl;
    builder.buildTask(domainReader.getMethods(), domainReader.getActions(), action_builder.getActions());
    builder.displayTask();

    procedural::TaskRecognition task_recognition;
    task_recognition.init(builder.getTask());
    procedural::ActionRecognition action_recognition;
    action_recognition.init(action_builder.getActions());
//    action_recognition.linkToTaskRecognition(&task_recognition);






    return 0;
}