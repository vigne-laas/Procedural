#include "procedural/old/core/Types/Task.h"

namespace procedural {
WordTable Task::task_types;

void Task::addMethods(Method* method)
{
    methods_.push_back(method);
    methods_.back()->close();
}
void Task::addArguments(const std::map<std::string, std::string>& arguments)
{
    arguments_ = arguments;
}
ResultFeedProcess_t<Method> Task::feed(StateMachine* state_machine)
{

    state_machine->setRead();
    std::cout << "\t\t --------------" << std::endl;
    std::cout << "\t\t\taction finished in Task recognition: " << state_machine->getName() << " "
              << state_machine->getAge() << "\n\n"
              << std::endl;
    ResultFeedProcess_t<Method> result;
    for (auto& method: methods_)
    {
        auto feed_result = method->feed(state_machine);
        result.combine(feed_result, method);
    }
    return result;
}

bool Task::feed(Task* task)
{
    for (auto& method: methods_)
        method->feed(task);
    return false;
}

std::map<std::string, Variable_t> Task::getVars()
{
    return std::map<std::string, Variable_t>();
}
void Task::clean()
{
    std::cout << "clean task " << name_ << std::endl;
}
void Task::updateMethod(MessageType type, Method* method)
{
    switch (type)
    {
        case MessageType::None:
            break;
        case MessageType::Update:
            updated_methods_.insert(method);
            break;
        case MessageType::Finished:
            finished_methods_.insert(method);
            break;
        case MessageType::Complete:
            finished_methods_.insert(method);
            break;
    }
}


} // procedural