#include <iostream>
#include "procedural/core/Action.h"

namespace procedural {
Action::Action(const std::string& name) : name_(name)
{

}

void Action::feed(const Fact& fact)
{
    for (auto it_network = networks_.begin(); it_network < networks_.end(); it_network++)
    {
        it_network->evolve(fact);
        if (it_network->isComplete())
        {
            complete_networks_.push_back(*it_network);
            networks_.erase(it_network);
        }
    }
    Network N = Network(facts_, name_);
    N.id_=networks_.size()-1;
    if (N.evolve(fact))
        networks_.push_back(N);

}

void Action::checkCompleteNetworks()
{
    for(auto it_network = networks_.begin(); it_network < networks_.end(); it_network++)
    {
        std::cout<<"finish network " << it_network->name_ << "_" << it_network->id_ << std::endl;
    }
}

const std::vector<std::vector<FactPattern>>& Action::getFacts()
{
    return facts_;
}

void Action::addFacts(const FactPattern& facts)
{

}

} // procedural