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
    auto sms = action->getFinishedStateMachine();
    for (auto sm: sms)
        for (auto var: sm->getLiteralVariables())


            return false;
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


} // procedural