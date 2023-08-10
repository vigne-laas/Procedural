#include "procedural/core/Graph/HTN/HtnState.h"

namespace procedural {
HTNState::HTNState(const std::string& name, int id) : id_(id), name_(name), initial_node_(false)
{}
HTNState* HTNState::evolve(Action* action)
{
    for (auto& next_action: nexts_actions_)
        if (next_action.first.match(action))
            return next_action.second;
    return nullptr;
}
HTNState* HTNState::evolve(Task* task)
{
    for (auto& next_task: nexts_task_)
        if (next_task.first.match(task))
            return next_task.second;
    return nullptr;
}
void HTNState::addTransition(const TransitionAction& transition, HTNState* next_state)
{
    nexts_actions_.emplace_back(transition,next_state);

}
void HTNState::addTransition(const TransitionTask& transition, HTNState* next_state)
{
    nexts_task_.emplace_back(transition,next_state);
}
void HTNState::linkVariables(std::map<std::string, Variable_t>& variables_)
{
    for (auto& pair: nexts_actions_)
        pair.first.linkVariables(variables_);
    for (auto& pair: nexts_task_)
        pair.first.linkVariables(variables_);
}
std::string HTNState::toString() const
{
    std::string msg = "State : " + name_ + "\n";
    msg += isFinalNode() ? "\tFinal Node \n" : "";
    msg += initial_node_ ? "\tInitial Node \n" : "";
    msg += "\tTransitions (" + std::to_string(nexts_actions_.size() + nexts_task_.size()) + "):";
    if (!nexts_actions_.empty())
        for (auto& pair_transition_state: nexts_actions_)
            msg += "\t[" + pair_transition_state.first.toString() + "]";
    if (!nexts_task_.empty())
        for (auto& pair_transition_state: nexts_task_)
            msg += "\t" + pair_transition_state.first.toString() + "]\n";

    return msg;
}
} // procedural