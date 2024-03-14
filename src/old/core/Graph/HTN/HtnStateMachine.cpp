#include "procedural/old/core/Graph/HTN/HtnStateMachine.h"

namespace procedural {

WordTable HTNStateMachine::types_table;

HTNStateMachine::HTNStateMachine(const std::string name, int id) : name_(name), id_(id),
                                                                   full_name_(name + "_" + std::to_string(id)),
                                                                   valid_(false),
                                                                   closed_(false),
                                                                   current_state_(nullptr),
                                                                   id_initial_state_(-1)
{

    variables_.emplace("self", full_name_);
    type_ = StateMachine::types_table.get(name);
}
bool HTNStateMachine::evolve(Action* action)
{
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(action);
    if (evolution == nullptr)
        return false;
    return true;
}
bool HTNStateMachine::evolve(Task* task)
{
    if ((valid_ && closed_) == false)
        return false;
    auto evolution = current_state_->evolve(task);
    if (evolution == nullptr)
        return false;
//    if (current_state_->getId() == id_initial_state_)
//        age_ = fact->getTimeStamp();
//    last_update_ = fact->getTimeStamp();

    return true;
}
void HTNStateMachine::addTransition(const HTNTransition_t& transition)
{
    addState(transition.origin_state);
    addState(transition.next_state);
    if (transition.type == TransitionType::Action)
    {
        TransitionAction transition_action(transition.id_substask);
        states_.at(transition.origin_state)->addTransition(transition_action, states_[transition.next_state]);
    }
    if (transition.type == TransitionType::Task)
    {
        TransitionTask transition_task(transition.id_substask);
        states_.at(transition.origin_state)->addTransition(transition_task, states_[transition.next_state]);
    }

}
bool HTNStateMachine::close()
{
    linkStateMachine();
    closed_ = true;
    processInitialState();
    valid_ = true;
    return valid_;
}
void HTNStateMachine::addState(int id_state)
{
    if (states_.find(id_state) == states_.end())
        states_.emplace(id_state, new HTNState(full_name_, id_state));
}
void HTNStateMachine::linkStateMachine()
{
    for (auto& state: states_)
    {
        state.second->linkVariables(variables_);
    }

}
void HTNStateMachine::processInitialState()
{
    std::unordered_set<uint32_t> id_states_nexts;
    for (auto& pair_states: states_)
    {
        for (auto& nexts_state: pair_states.second->getNextsActions())
            id_states_nexts.insert(nexts_state.second->getId());
        for (auto& nexts_state: pair_states.second->getNextsTasks())
            id_states_nexts.insert(nexts_state.second->getId());
    }

    std::unordered_set<int> result;
    for (auto& state: states_)
    {
        if (id_states_nexts.find(state.first) == id_states_nexts.end())
            result.insert(state.first);
    }

    int nb_initial_state = result.size();
    if (result.size() == 0)
    {
        throw NoInitialHTNStateStateMachineException();
    }
    else if (nb_initial_state > 1)
    {
        std::unordered_set<HTNState*> invalid_states;
        for (auto res: result)
            invalid_states.insert(states_.at(res));

        throw MultiInitialStateHTNStateMachineException(invalid_states);
    }
    else
    {
        id_initial_state_ = *result.begin();
        current_state_ = states_.at(id_initial_state_);
    }

}
} // procedural