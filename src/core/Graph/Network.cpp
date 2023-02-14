#include <iostream>
#include "procedural/core/Graph/Network.h"

namespace procedural {

Network::Network(const std::string& name, int id) : name_(name),
                                                    id_(id),
                                                    closed_(false),
                                                    valid_(false)
{
    full_name_ = name_ + " " + std::to_string(id);
}

bool Network::evolve(const Fact& fact)
{
    auto evolution = current_state_->evolve(fact);
    if (evolution == nullptr)
        return false;
    current_state_ = evolution;
    return true;
}

bool Network::addTransition(const PatternTransition_t& pattern)
{  
    if(not closed_)
    {
        addState(pattern.origin_state);
        addState(pattern.next_state);

        insertVariable(pattern.fact->getVarSubject());
        insertVariable(pattern.fact->getVarObject());
        
        Transition t = Transition(*(pattern.fact));
        states_[pattern.origin_state]->addTransition(t, states_[pattern.next_state]);
        return true;
    }
    else
    {
        // Maybe raise an error rather than printing text
        std::cout << "Network closed impossible to add new transition" << std::endl;
        return false;
    }
}

void Network::addState(int id_state)
{
    if(states_.find(id_state) == states_.end())
        states_.emplace(id_state, new State(name_, id_state));
}

bool Network::closeNetwork()
{
    linkNetwork();
    valid_ = processInitialState();
    closed_ = true;
    return valid_;
}

std::string Network::toString()
{
    std::string res;
    for (auto& state : states_)
    {
        if(res != "")
            res += "\n";
        res += "id : " + std::to_string(state.first) + " state :" + state.second->toString();
    }
    return res;
}

Network *Network::clone(int new_id)
{
    // We may need to test if the original network is closed and valid first
    Network* N = new Network(name_+"_child", new_id);
    N->variables_ = variables_;

    for(auto& state : states_)
        N->addState(state.first);
    
    for(auto& state : N->states_)
    {
        for(auto& pair_transition : states_.at(state.first)->getNexts())
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
        std::cout << "key : " << var.first << " => " << std::to_string(var.second.value) << std::endl;
}

void Network::linkNetwork()
{
    for (auto& state: states_)
    {
        state.second->linkVariables(variables_);
        //state.expandTransitions();
    }
}

void Network::insertVariable(const std::string& variable)
{
    variables_.emplace(variable,variable);
}

bool Network::processInitialState()
{
    std::unordered_set<int> id_states_nexts;
    for(auto& pair_states : states_)
    {
        for(auto& nexts_state : pair_states.second->getNexts())
            id_states_nexts.insert(nexts_state.second->getId());
    }

    std::unordered_set<int> result;
    for(auto& state : states_)
    {
        if(id_states_nexts.find(state.first) == id_states_nexts.end())
            result.insert(state.first);
    }

    int nb_initial_state  = result.size();
    if(result.size() == 0)
    {
        // Maybe raise an error rather than printing text
        std::cerr << "No initial state detected." << std::endl;
        return false;
    }
    else if(nb_initial_state > 1)
    {
        // Maybe raise an error rather than printing text
        std::cerr << "Multiple initial states detected." << std::endl;
        std::cerr << "State detected as initial are : " << std::endl;
        for(auto& id : result)
            std::cerr << states_.at(id)->toString() << std::endl;

        return false;
    }
    else
    {
        id_initial_state_ = *result.begin();
        current_state_ = states_.at(id_initial_state_);
        return true;
    }
}

} // namespace procedural