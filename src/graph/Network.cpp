#include <iostream>
#include "procedural/graph/Network.h"

namespace procedural {

Network::Network(const std::string& name,int id)
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

void Network::updateVariables(const Fact& fact, const Transition& update_transition)
{
//    variables_map_.at(update_transition.getVarObject()) = fact.getObject();
//    variables_map_.at(update_transition.getVarSubject()) = fact.getSubject();
    for (auto& state: graph_)
    {
//        for (auto& transition: state.getTransitions())
//        {
//            transition.checkUpdate(this);
//        }
    }
}

void Network::addTransition(const PatternTransition_t& pattern)
{  
    addState(pattern.origin_state);
    addState(pattern.next_state);
    checkVar(*(pattern.fact));

    if(pattern.is_initial_state)
    {
        current_state_ = states_[pattern.origin_state];
        id_initial_state_ = pattern.origin_state;
    }
    
    Transition t = Transition(*(pattern.fact));
    states_[pattern.origin_state]->addTransition(t, states_[pattern.next_state]);
    linkNetwork();
}

void Network::addState(int id_state)
{
    if(states_.find(id_state) == states_.end())
        states_.emplace(id_state, new State(name_, id_state));
}

std::string Network::map2String()
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

Network *Network::clone()
{
    Network* N = new Network(name_+"_copy", id_ + 1);
    N->literal_variables_ = literal_variables_;
    // N->graph_ = graph_;
    for(auto& state : states_)
        N->addState(state.first);
    N->variables_ = variables_;
    
    for(auto& state : N->states_)
    {
        std::cout << "State id : " << state.second->getId() << std::endl;
        state.second->linkTransitions(states_, N->states_);
    }
    N->linkNetwork();
    N->id_initial_state_ = id_initial_state_;
    N->current_state_ = N->states_.at(N->id_initial_state_);
    return N;
}

void Network::displayNetwork()
{
    for (auto& state: graph_)
        std::cout << state.toString() << "\n\n" << std::endl;
}

void Network::displayVariables()
{
    for (auto& var: variables_)
        std::cout << "key : " << var.literal << " => " << std::to_string(var.value) << std::endl;
}

void Network::linkNetwork()
{
    /* for (auto& state: graph_)
    {
        state.linkTransitions(variables_);
//        state.expandTransitions();
    }
    graph_.front().setInitialNode();
    current_state_ = &(graph_.front()); */
    for (auto& state: states_)
    {
        state.second->linkVariables(variables_);
        // std::cout << state.second->getId() << std::endl();
//        state.expandTransitions();
    }
    // graph_.front().setInitialNode();
    // current_state_ = &(graph_.front());

}

void Network::checkVar(const FactPattern& pattern)
{
    if (literal_variables_.find(pattern.getVarSubject()) == literal_variables_.end())
    {
        literal_variables_.insert(pattern.getVarSubject());
        variables_.emplace_back(pattern.getVarSubject());
//        std::cout << "add var : " << pattern.getVarSubject() << std::endl;
    }
    if (literal_variables_.find(pattern.getVarObject()) == literal_variables_.end())
    {
        literal_variables_.insert(pattern.getVarObject());
        variables_.emplace_back(pattern.getVarObject());
//        std::cout << "add var : " << pattern.getVarObject() << std::endl;

    }

}

} // namespace procedural