#include <iostream>
#include "procedural/core/Types/Action.h"

namespace procedural {

Action::Action(const std::string& name) : name_(name), flag_(false)
{}

bool Action::addPatterns(const SpecializedAction& pattern)
{
    if (pattern.isValid())
    {
        patterns_.push_back(pattern);
        return true;
    } else
        return false;
}

void Action::feed(Fact* fact, TimeStamp_t currentTimestamp)
{

    for (auto& pattern: patterns_)
        if ((currentTimestamp - fact->getTimeStamp()) < pattern.getTtl())
            pattern.feed(fact);
//        else
//        {
//            std::cout << "rejected fact : " << fact->toString() << "for  :" << pattern.getName() << std::endl;
//            std::cout << "delta _t =" << currentTimestamp - fact->getTimeStamp() << " / ttl : " << pattern.getTtl()
//                      << std::endl;
//        }
}

std::set<uint32_t> Action::checkCompleteNetworks(TimeStamp_t currentTimestamp)
{
    std::set<uint32_t> set_valid_facts;
    if (flag_ == false)
    {
        for (auto& pattern: patterns_)
        {
            std::set<uint32_t> temp_set = pattern.checkNetwork(currentTimestamp);
            set_valid_facts.insert(temp_set.begin(), temp_set.end());
            if (temp_set.empty() == false)
            {
                std::unordered_set<Network*> temp = pattern.getCompleteNetwork();
                complete_networks_.insert(temp.begin(), temp.end());
            }

        }
        for (auto& pattern: patterns_)
            pattern.cleanInvolve(set_valid_facts);
        flag_ = true;
    }
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
    flag_ = false;
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
//        pattern.clean();
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
    for (auto& pattern: patterns_)
        evolve |= pattern.checksubAction(action);
    if (evolve)
        flag_ = false;
    return evolve;
}
bool Action::checkNewExplanation()
{
    bool res = false;
    if (flag_ == false)
    {
        for (auto& specializedAction: patterns_)
            res |= specializedAction.checkNewUpdatedSubNetwork();
    }

    return res;
}
std::vector<Network*> Action::getNewExplanation()
{
    std::vector<Network*> res;
    for (auto& specializedAction: patterns_)
    {
        auto nets = specializedAction.getUpdatedNetworks();
        res.insert(res.end(), nets.begin(), nets.end());
    }
    return res;
}
double Action::maxTtl()
{
    double res;
    for (auto& action: patterns_)
        if (action.getTtl() > res)
            res = action.getTtl();
    return res;
}


} // namespace procedural