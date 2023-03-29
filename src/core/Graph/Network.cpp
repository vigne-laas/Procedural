#include <iostream>
#include <set>
#include "procedural/core/Graph/Network.h"

namespace procedural {


Network::Network(const std::string& name, int id) : name_(name),
                                                    id_(id),
                                                    closed_(false),
                                                    valid_(false),
                                                    age_(0)
{
    full_name_ = name_ + " " + std::to_string(id);
    variables_.emplace("self", full_name_);

}

bool Network::evolve(Fact* fact)
{
    if ((valid_ && closed_) == false)
        return false;
    age_++;
    auto evolution = current_state_->evolve(fact);

    if (evolution == nullptr)
        return false;

    current_state_ = evolution;
    age_ = 0;
    id_facts_involve.push_back(fact->getId()); // prepare to id on facts.

    return true;
}

bool Network::addTransition(const PatternTransition_t& pattern)
{
    if (closed_ == false)
    {
        addState(pattern.origin_state);
        addState(pattern.next_state);

        insertVariable(pattern.fact->getVarSubject());
        insertVariable(pattern.fact->getVarObject());

        Transition t = Transition(*(pattern.fact));
        states_[pattern.origin_state]->addTransition(t, states_[pattern.next_state]);
        return true;
    } else
        return false;

}

bool Network::addDescription(const ActionDescription_t& des)
{
    descriptions_.emplace_back(des, variables_);
    return true;
}

bool Network::closeNetwork()
{
    linkNetwork();
    closed_ = true;
    processInitialState();
    valid_ = true;
    return valid_;
}

std::string Network::toString()
{
    std::string res;
    for (auto& state: states_)
    {
        if (res != "")
            res += "\n";
        res += "id : " + std::to_string(state.first) + " " + state.second->toString();
    }
    res += "current state : " + current_state_->toString();
    return res;
}

Network* Network::clone(int new_id)
{
    if ((valid_ && closed_) == false)
        return nullptr;

    Network* N = new Network(name_ + "_child", new_id);
    N->variables_ = variables_;
    N->variables_.at("self").literal = N->getName();
    N->descriptions_ = descriptions_;

    for (auto& state: states_)
        N->addState(state.first);

    for (auto& state: N->states_)
    {
        for (auto& pair_transition: states_.at(state.first)->getNexts())
        {
            Transition t = pair_transition.first;
            state.second->addTransition(t, N->states_.at(pair_transition.second->getId()));
        }
    }

    N->closeNetwork();
    return N;
}

void Network::displayVariables()
{
    for (auto& var: variables_)
        std::cout << "key : " << var.first << " => " << var.second.toString() << std::endl;
}

std::string Network::describe(bool expl)
{
    if ((valid_ && closed_) == false)
        return "";
    std::string msg = "\t";
    for (auto& description: descriptions_)
    {
        if (expl)
            msg += description.explainExplicit() + " / ";
        else
            msg += description.explain() + " / ";
    }


    return msg;
}

bool Network::involveFacts(const std::set<uint32_t>& facts)
{
    for (auto id_fact: id_facts_involve)
        if (facts.find(id_fact) == facts.end())
            return false;
    return true;
}

/* ------------------------------ private part ------------------------------ */

void Network::addState(int id_state)
{
    if (states_.find(id_state) == states_.end())
        states_.emplace(id_state, new State(getName(), id_state));
}

void Network::linkNetwork()
{
    for (auto& state: states_)
    {
        state.second->linkVariables(variables_);
        //state.expandTransitions();
    }
    for (auto& des: descriptions_)
        des.linkVariables(variables_);
}

void Network::insertVariable(const std::string& variable)
{
    variables_.emplace(variable, variable);
}

void Network::processInitialState()
{
    std::unordered_set<int> id_states_nexts;
    for (auto& pair_states: states_)
    {
        for (auto& nexts_state: pair_states.second->getNexts())
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
        throw NoInitialStateNetworkException();
    } else if (nb_initial_state > 1)
    {
        // Maybe raise an error rather than printing text
        std::unordered_set<State*> invalid_states;
        for (auto res: result)
        {
            invalid_states.insert(states_.at(res));
        }
        throw MultiInitialStateNetworkException(invalid_states);
    } else
    {
        id_initial_state_ = *result.begin();
        current_state_ = states_.at(id_initial_state_);
    }
}

} // namespace procedural