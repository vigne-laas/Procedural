#include "procedural/core/Graph/State.h"

#include <iostream>

namespace procedural {

State::State(const std::string& name, int id) : name_(name + "_" + std::to_string(id)),
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

void State::linkTransitions(std::vector<Variable_t>& variables_)
{
    for (auto& pair: nexts_)
        pair.first.linkVariables(variables_);
}

void State::expandTransitions()
{
    for(auto& next : nexts_)
        next.first.expandProperty();
}

std::string State::toString()
{
    std::string msg = "State : " + name_ + "\n";
    msg += isFinalNode() ? "Final Node \n" : "";
    msg += initial_node_ ? "Initial Node \n" : "";
    msg += "\t nombre de transition : " + std::to_string(nexts_.size()) + "\n";
    if (!nexts_.empty())
        for (auto& pair_transition_state: nexts_)
        {
            msg += "\t\t -  " + pair_transition_state.first.toString();
        }
    return msg;
}

} // namespace procedural