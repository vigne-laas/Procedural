#include "procedural/core/Graph/State.h"

#include <iostream>
#include <ontologenius/clients/ontologyClients/ObjectPropertyClient.h>

namespace procedural {

State::State(const std::string& name, int id) : id_(id),
                                                name_(name + "_" + std::to_string(id)),
                                                initial_node_(false),
                                                has_timeout_transition(false)
{}

State* State::evolve(Fact* fact)
{
    for (auto& pair: nexts_facts_)
        if (pair.first.match(fact))
            return pair.second;

    return nullptr;
}

std::pair<State*, TransitionStateMachine*> State::evolve(StateMachine* state_machine)
{
    for (auto& pair: nexts_state_machines_)
        if (pair.first.match(state_machine))
            return std::make_pair(pair.second, &pair.first);

    return std::make_pair(nullptr, nullptr);
}

void State::addTransition(const TransitionFact& transition, State* next_state)
{
    nexts_facts_.emplace_back(transition, next_state);
}

void State::addTransition(const TransitionStateMachine& transition, State* next_state)
{
    nexts_state_machines_.emplace_back(transition, next_state);
}

void State::linkVariables(std::map<std::string, Variable_t>& variables_)
{
    for (auto& pair: nexts_facts_)
        pair.first.linkVariables(variables_);
    for (auto& pair: nexts_state_machines_)
        pair.first.linkVariables(variables_);
}


void State::expandTransitions(ObjectPropertyClient* object_client)
{
    for (auto& next: nexts_facts_)
        next.first.expandProperty(object_client);

}

std::string State::toString() const
{
    std::string msg = "State : " + name_ + "\n";
    msg += isFinalNode() ? "\tFinal Node \n" : "";
    msg += initial_node_ ? "\tInitial Node \n" : "";
    msg += "\tTransitions (" + std::to_string(nexts_facts_.size() + nexts_state_machines_.size()) + "):";
    if (!nexts_facts_.empty())
        for (auto& pair_transition_state: nexts_facts_)
            msg += "\t[" + pair_transition_state.first.toString() + "]";
    if (!nexts_state_machines_.empty())
        for (auto& pair_transition_state: nexts_state_machines_)
            msg += "\t" + pair_transition_state.first.toString() + "]\n";

    return msg;
}
void State::addTimeoutTransition()
{
    has_timeout_transition = true;
}


} // namespace procedural