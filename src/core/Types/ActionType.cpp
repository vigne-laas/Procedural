#include <iostream>
#include "procedural/core/Types/ActionType.h"

namespace procedural {

ActionType::ActionType(const std::string& name) : name_(name), flag_(false)
{}

bool ActionType::addPatterns(const Action& pattern)
{
    if (pattern.isValid())
    {
        patterns_.push_back(pattern);
        return true;
    } else
        return false;
}

bool ActionType::feed(Fact* fact, TimeStamp_t currentTimestamp)
{
    bool evolve = false;
    for (auto& pattern: patterns_)
        if ((currentTimestamp - fact->getTimeStamp()) <= pattern.getTtl())
            evolve |= pattern.feed(fact);
//        else
//        {
            // std::cout << "rejected fact : " << fact->toString() << " for  :" << pattern.getName() << std::endl;
//            std::cout << "delta _t =" << currentTimestamp - fact->getTimeStamp() << " / ttl : " << pattern.getTtl()
//                      << std::endl;
//        }
    return evolve;
}

std::set<uint32_t> ActionType::checkCompleteNetworks(TimeStamp_t currentTimestamp)
{
//    std::cout << "complete_networks_ set beginning " << std::endl;
//    for(auto net : complete_networks_)
//        std::cout << net->getName() << std::endl;
    std::set<uint32_t> set_valid_facts;
    if (flag_ == false)
    {
        for (auto& pattern: patterns_)
        {
            std::set<uint32_t> temp_set = pattern.checkNetwork(currentTimestamp);
            set_valid_facts.insert(temp_set.begin(), temp_set.end());
            if (temp_set.empty() == false)
            {
                std::unordered_set<StateMachine*> temp = pattern.getCompleteNetwork();
//                std::cout << "temp set " << std::endl;
//                for(auto net : temp)
//                    std::cout << net->getName() << std::endl;
                complete_networks_.insert(temp.begin(), temp.end());
//                std::cout << "complete_networks_ set " << std::endl;
//                for(auto net : complete_networks_)
//                    std::cout << net->getName() << std::endl;
//                std::cout << "complete net at :" << currentTimestamp << " from : "<<pattern.getName() <<" : \n";
//                for(auto net : complete_networks_)
//                    std::cout << "\t " <<net->getName() << std::endl;
            }

        }
        for (auto& pattern: patterns_)
            pattern.cleanInvolve(set_valid_facts);
        flag_ = true;
    }
    return set_valid_facts;
}

void ActionType::displayCurrentState()
{
//    for(auto& pattern : patterns_)
//        for(auto& net : pattern.networks_) // TODO networks_ should not be public
//            std::cout << net->getCurrentState()->toString() << std::endl;
}

void ActionType::clean()
{
    flag_ = false;
    for (auto& pattern: patterns_)
    {
        pattern.clean();
    }
    complete_networks_.clear();
}

void ActionType::cleanPatterns(std::set<uint32_t> set_id)
{
    for (auto& pattern: patterns_)
    {
        pattern.cleanInvolve(set_id);
//        pattern.clean();
    }
}

std::string ActionType::toString()
{
    std::string res;
    for (auto& pattern: patterns_)
        res += pattern.toString() + "\n";
    return res;
}

std::string ActionType::currentState(bool shortVersion)
{
    std::string res;
    for (auto& pattern: patterns_)
        res += pattern.currentState(shortVersion) + "\n";
    return res;
}
bool ActionType::checkSubAction(ActionType* action)
{
    bool evolve = false;
    for (auto& pattern: patterns_)
        evolve |= pattern.checksubAction(action);
    if (evolve)
        flag_ = false;
    return evolve;
}
bool ActionType::checkNewExplanation()
{
    bool res = false;
    if (flag_ == false)
    {
        for (auto& specializedAction: patterns_)
            res |= specializedAction.checkNewUpdatedSubNetwork();
    }

    return res;
}
std::vector<StateMachine*> ActionType::getNewExplanation()
{
    std::vector<StateMachine*> res;
    for (auto& specializedAction: patterns_)
    {
        auto nets = specializedAction.getUpdatedNetworks();
        res.insert(res.end(), nets.begin(), nets.end());
    }
    return res;
}
double ActionType::maxTtl()
{
    double res;
    for (auto& action: patterns_)
        if (action.getTtl() > res)
            res = action.getTtl();
    return res;
}


} // namespace procedural