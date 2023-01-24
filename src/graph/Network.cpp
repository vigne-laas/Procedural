#include <iostream>
#include "procedural/graph/Network.h"

namespace procedural {

Network::Network(const std::vector<std::vector<FactPattern>>& patterns, const std::string& name) : name_(name),
                                                                                                   current_state_(
                                                                                                           nullptr)
{
    graph_.resize(patterns.size() + 1, State(name_));
    for (auto index_pattern = 0; index_pattern < patterns.size(); index_pattern++)
    {
        graph_[index_pattern].id_ = index_pattern;
        if (patterns[index_pattern].size() == 1)
            addTransitionIndex(patterns[index_pattern].front(), index_pattern);
        else
        {
            for (auto& or_pattern: patterns[index_pattern])
            {
                addTransitionIndex(or_pattern, index_pattern);
            }
        }
    }
//    initialTransition_ = graph_.front().getTransitions();
//    graph_[0].setInitialNode();
    graph_.back().id_ = graph_.size() - 1;
    current_state_ = &graph_.front();
//    graph_[graph_.size()-1].id_ = graph_.size()-1;
}

void Network::addTransitionIndex(const FactPattern& pattern, int32_t index)
{
//    std::cout << pattern.toString() << std::endl;
//    std::cout<<"Add in map var subject : "<<pattern.getVarSubject()<<std::endl;
//    std::cout<<"Add in map var object : "<<pattern.getVarObject()<<std::endl;

    variables_map_.insert({pattern.getVarObject(), -1});
    variables_map_.insert({pattern.getVarSubject(), -1});
    Transition t = Transition(pattern);

    if (index < graph_.size())
        t.setNextState(&(graph_[index + 1]));
    graph_[index].addTransition(t);
    if (pattern.isRequired())
    {
        for (auto i = 0; i < index; i++)
        {
            graph_[i].addTransition(t);
        }
    }
}

void Network::displayNetwork()
{
    for (auto& state: graph_)
    {
        std::cout << state.toString() << "\n\n" << std::endl;

    }
}

void Network::displayVariables()
{
    for (auto& var: variables_map_)
    {
        std::cout << "key : " << var.first << " => " << std::to_string(var.second) << std::endl;
    }
}

bool Network::evolve(const Fact& fact)
{

    for (auto& transition: current_state_->getTransitions())
    {
        auto evolution = transition.evolve(fact);
        if (evolution != nullptr)
        {
//            std::cout<<"evolution"<<std::endl;
            current_state_ = evolution;
            updateVariables(fact,transition);
            if(current_state_->isFinalNode())
                std::cout<< "action detected : " << name_ <<std::endl;
            return true;
        }
    }
    return false;
}

void Network::updateVariables(const Fact& fact, const Transition& update_transition)
{
    variables_map_.at(update_transition.getVarObject()) = fact.getObject();
    variables_map_.at(update_transition.getVarSubject()) = fact.getSubject();
    for (auto& state: graph_)
    {
        for(auto& transition: state.getTransitions())
        {
            transition.checkUpdate(this);
        }
    }
}


}

