#include "procedural/core/Graph/State.h"

#include <iostream>

namespace procedural {

State::State(const std::string& name, int id) : id_(id),
                                                name_(name + "_" + std::to_string(id)),
                                                initial_node_(false)
{}

State* State::evolve(const Fact& fact)
{
    for (auto& pair: nexts_)
        if (pair.first.matchFact(fact))
            return pair.second;

    return nullptr;
}

void State::addTransition(const Transition& transition, State* next_state)
{
    nexts_.emplace_back(transition, next_state);
}

void State::linkVariables(std::vector<Variable_t>& variables_)
{
    for (auto& pair: nexts_)
        pair.first.linkVariables(variables_);
}

void State::linkTransitions(const std::map<int, State*>& map_state_mother, const std::map<int, State*>& map_state_me)
{
    // TODO pass it to Network
    for(auto& pair_transition : map_state_mother.at(id_)->nexts_)
    {
        Transition t = pair_transition.first;
        nexts_.emplace_back(t, map_state_me.at(pair_transition.second->getId()));
    } 
}

void State::expandTransitions()
{
    for(auto& next : nexts_)
        next.first.expandProperty();
}

std::string State::toString() const
{
    std::string msg = "State : " + name_ + "\n";
    msg += isFinalNode() ? "\tFinal Node \n" : "";
    msg += initial_node_ ? "\tInitial Node \n" : "";
    msg += "\tTransitions (" + std::to_string(nexts_.size()) + "):";
    if (!nexts_.empty())
        for (auto& pair_transition_state: nexts_)
            msg += "\t[" + pair_transition_state.first.toString() + "]";

    return msg;
}

} // namespace procedural