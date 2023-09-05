#include "procedural/core/Graph/Transitions/TransitionTask.h"
#include "procedural/core/Types/Task.h"

namespace procedural {
TransitionTask::TransitionTask(uint32_t task_id, int next_state, const std::map<std::string, std::string>& arguments)
        : task_id_(task_id), id_next_state_(next_state), arguments_(arguments)
{
    for (const auto& arg: arguments)
        variables_.insert(std::make_pair(arg.first, nullptr));
}
TransitionTask::TransitionTask(const TransitionTask& t, int next_state):task_id_(t.task_id_),id_next_state_(next_state),arguments_(t.arguments_)
{
    for (const auto& arg: arguments_)
        variables_.insert(std::make_pair(arg.first, nullptr));
}
void TransitionTask::linkVariables(std::map<std::string, Variable_t>& variables)
{
    for (auto& pair: variables_)
        pair.second = &(variables.at(pair.first));
}
bool TransitionTask::match(Task* task)
{
    if(task->getId() == task_id_)
        if(checkArgs(task))
            return true;
    return false;
}
std::string TransitionTask::toString() const
{
    std::string res =
            "Task Transitions type : " + std::to_string(task_id_) + "(" + Task::task_types.get(task_id_) + ") to " +
            std::to_string(id_next_state_) + " \n";
    if (!variables_.empty())
    {
        res += "Variables : \n";
        for (auto& var: variables_)
            res += "\t" + var.first + " : " + var.second->toString() + "\n";
    }

    return res;
}
void TransitionTask::setOntologyClient(onto::IndividualClient* indiv_client)
{

}
bool TransitionTask::checkArgs(Task* task)
{
//    for(const auto& method  : task->getFinishedMethods())
//        method->getVars()
    return false;
}

} // procedural