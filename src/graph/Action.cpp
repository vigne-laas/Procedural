#include <iostream>
#include "procedural/graph/Action.h"

namespace procedural {

Action::Action(const std::string& name) : name_(name)
{

}
void Action::addPatterns(PatternRecognition& pattern)
{
    pattern.buildNetwork();
    patterns_.push_back(pattern);
}

void Action::close()
{
    for(auto& pattern : patterns_)
        pattern.buildNetwork();
}



void Action::feed(const Fact& fact)
{
    for(auto& pattern : patterns_)
        pattern.feed(fact);
}

void Action::checkCompleteNetworks()
{
    for(auto& pattern : patterns_)
    {
        pattern.checkNetwork();
    }
}

void Action::displayCurrentState()
{
    for(auto& pattern : patterns_)
    {
        for(auto& net : pattern.networks_)
        {
            std::cout << net->getCurrentState()->toString() << std::endl;
        }
    }

}
std::string Action::toString()
{
    std::string res;
    for(auto& pattern : patterns_)
        res += pattern.toString()+"\n";
    return res;

}
// const std::vector<std::vector<FactPattern>>& Action::getFacts()
//{
//     return facts_;
// }





} // namespace procedural