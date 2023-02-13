#include <iostream>
#include "procedural/graph/Network.h"

namespace procedural {

Network::Network(const std::string& name,int id):closed_(false)
{
    name_ = name;
    id_ = id;
}

bool Network::evolve(const Fact& fact)
{
    auto evolution = current_state_->evolve(fact);
    if (evolution == nullptr)
        return false;
    current_state_ = evolution;
    return true;
}

void Network::addTransition(const PatternTransition_t& pattern)
{  
    if(not closed_)
    {
        addState(pattern.origin_state);
        addState(pattern.next_state);

        insertVariable(pattern.fact->getVarSubject());
        insertVariable(pattern.fact->getVarObject());
        
        Transition t = Transition(*(pattern.fact));
        states_[pattern.origin_state]->addTransition(t, states_[pattern.next_state]);
    }
    else
    {
        std::cout << "Network closed impossible to add new transition" << std::endl;
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
    closed_ = processInitialState();   
    return closed_;
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
    // Network* N = new Network(name_+"_copy", new_id); // id_ + 1 will fail as we consider one mother creating all the childs
    Network* N = new Network(name_+"_child", new_id); // id_ + 1 will fail as we consider one mother creating all the childs
    N->literal_variables_ = literal_variables_;
    N->variables_ = variables_;

    for(auto& state : states_)
        N->addState(state.first);
    
    for(auto& state : N->states_)
    {
        std::cout << "State id : " << state.second->getId() << std::endl;
        for(auto& pair_transition : states_.at(state.first)->getNexts())
        {
            Transition t = pair_transition.first;
            state.second->addTransition(t,N->states_.at(pair_transition.second->getId()));
        } 
        
    }

    N->closeNetwork();
    return N;
}

void Network::displayVariables()
{
    for (auto& var: variables_)
        std::cout << "key : " << var.literal << " => " << std::to_string(var.value) << std::endl;
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
    if (literal_variables_.find(variable) == literal_variables_.end())
    {
        literal_variables_.insert(variable);
        variables_.emplace_back(variable);
    }
}

bool Network::processInitialState()
{
    std::vector<int> id_states;
    std::vector<int> id_states_nexts;
    for(auto& pair_states :states_)
    {
        if(std::find(id_states_nexts.begin(), id_states_nexts.end(), pair_states.first) == id_states_nexts.end())
        {
            id_states.push_back(pair_states.first);
            for(auto& nexts_state : pair_states.second->getNexts())
            {
                id_states_nexts.push_back(nexts_state.second->getId());
            }
        }
    }
    int nb_initial_state  = id_states.size();
    // std::cout << "Number of initial state : " << nb_initial_state << std::endl;
    if(nb_initial_state!=1)
    {
        std::cerr << "Incorect number of initial state detected : " << nb_initial_state << " must be 1" << std::endl;
        std::cerr << "State detected as initial : "<< std::endl;
        for(auto& id : id_states)
        {
            std::cerr<< states_.at(id) << std::endl;
        }
        return false;
    }
    else
    {
        id_initial_state_ = id_states.front();
        current_state_ = states_.at(id_initial_state_);
        return true;
        // std::cout << "Initial state find : " << current_state_->toString() << std::endl;
    }
}
} // namespace procedural