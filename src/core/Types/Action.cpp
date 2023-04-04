#include <iostream>
#include "procedural/core/Types/Action.h"

namespace procedural {

Action::Action(const std::string& name) : name_(name)
{}

bool Action::addPatterns(const PatternRecognition& pattern)
{
    if (pattern.isValid())
    {
        patterns_.push_back(pattern);
        return true;
    }
    else
        return false;
}

void Action::feed(Fact* fact)
{
    for (auto& pattern: patterns_)
        pattern.feed(fact);
}

std::set<uint32_t> Action::checkCompleteNetworks()
{
    std::set<uint32_t> set_valid_facts;
    for (auto& pattern: patterns_)
    {
        std::set<uint32_t> temp_set = pattern.checkNetwork();
        set_valid_facts.insert(temp_set.begin(), temp_set.end());
        if(temp_set.empty() == false)
        {
            std::unordered_set<Network*> temp = pattern.getCompleteNetwork();
            complete_networks_.insert(temp.begin(),temp.end());
        }

    }
    for (auto& pattern: patterns_)
        pattern.cleanInvolve(set_valid_facts);

    return set_valid_facts;
}

void Action::displayCurrentState()
{
//    for(auto& pattern : patterns_)
//        for(auto& net : pattern.networks_) // TODO networks_ should not be public
//            std::cout << net->getCurrentState()->toString() << std::endl;
}
void Action::clean()
{
    for (auto& pattern: patterns_)
    {
        pattern.clean();
    }
}

void Action::cleanPatterns(std::set<uint32_t> set_id)
{
    for (auto& pattern: patterns_)
    {
        pattern.cleanInvolve(set_id);
        pattern.clean();
    }
}

std::string Action::toString()
{
    std::string res;
    for (auto& pattern: patterns_)
        res += pattern.toString() + "\n";
    return res;
}

std::string Action::currentState(bool shortVersion)
{
    std::string res;
    for (auto& pattern: patterns_)
        res += pattern.currentState(shortVersion) + "\n";
    return res;
}
bool Action::checkSubAction(Action* action)
{
    bool evolve = false;
    for(auto& pattern : patterns_)
        evolve &= pattern.checksubAction(action);
    return evolve;
}


} // namespace procedural