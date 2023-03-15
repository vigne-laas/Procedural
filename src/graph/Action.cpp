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
    for(auto& pattern : patterns_)
        pattern.checkNetwork();
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