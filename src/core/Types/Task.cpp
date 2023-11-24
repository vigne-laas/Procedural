#include "procedural/core/Types/Task.h"

namespace procedural {
WordTable Task::task_types;

void Task::addMethods(const Method& method)
{
    methods_.push_back(method);
    methods_.back().close();
}
void Task::addArguments(const std::map<std::string, std::string>& arguments)
{
    arguments_ = arguments;
}
bool Task::feed(Action* action)
{
//    auto sms = action->getFinishedStateMachine();
//    for (auto sm: sms)
//        for (auto var: sm->getLiteralVariables())
//
//
//            return false;
}

bool Task::feed(Task* task)
{
    for (auto& method: methods_)
        method.feed(task);
    return false;
}
bool Task::feed(ActionMethod* action_method)
{
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
void Task::notify(MessageType type)
{
     for (const auto& obs: list_observer_)
        obs->updateTask(type, this);
}



} // procedural