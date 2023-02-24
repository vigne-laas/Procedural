#include <iostream>
#include "procedural/graph/Action.h"

namespace procedural {

Action::Action(const std::string& name) : name_(name)
{}

bool Action::addPatterns(const PatternRecognition& pattern)
{
    if(pattern.isValid())
    {
        patterns_.push_back(pattern);
        return true;
    }
    else
        return false;
}

void Action::feed(Fact* fact)
{
    for(auto& pattern : patterns_)
        pattern.feed(fact);
}

void Action::checkCompleteNetworks()
{
    std::vector<std::vector<uint32_t>> list_valid_facts;
    for(auto& pattern : patterns_)
        for(auto& list : pattern.checkNetwork())
        {
            list_valid_facts.push_back(list);
        }
    for(auto& id_facts : list_valid_facts)
    {
        for(auto& pattern : patterns_)
            pattern.cleanInvolve(id_facts);
    }
}

void Action::displayCurrentState()
{
    /*for(auto& pattern : patterns_)
        for(auto& net : pattern.networks_) // TODO networks_ should not be public 
            std::cout << net->getCurrentState()->toString() << std::endl;*/
}

std::string Action::toString()
{
    std::string res;
    for(auto& pattern : patterns_)
        res += pattern.toString() + "\n";
    return res;
}

} // namespace procedural