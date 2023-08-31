#include <iostream>
#include "procedural/core/Types/ActionMethod.h"

namespace procedural {

ActionMethod::ActionMethod(const std::string& name) : name_(name), flag_(false)
{}

bool ActionMethod::addActions(const Action& action)
{
    if (action.isValid())
    {
        actions_.push_back(action);
        return true;
    } else
        return false;
}

bool ActionMethod::feed(Fact* fact, TimeStamp_t currentTimestamp)
{
    bool evolve = false;
    for (auto& pattern: actions_)
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

std::set<uint32_t> ActionMethod::checkCompleteStateMachines(TimeStamp_t currentTimestamp)
{
//    std::cout << "complete_state_machine_ set beginning " << std::endl;
//    for(auto net : complete_networks_)
//        std::cout << net->getName() << std::endl;
    std::set<uint32_t> set_valid_facts;
    if (flag_ == false)
    {
        for (auto& pattern: actions_)
        {
            std::set<uint32_t> temp_set = pattern.checkStateMachine(currentTimestamp);
            set_valid_facts.insert(temp_set.begin(), temp_set.end());
            if (temp_set.empty() == false)
            {
                std::unordered_set<StateMachine*> temp = pattern.getFinishedStateMachine();
//                std::cout << "temp set " << std::endl;
//                for(auto net : temp)
//                    std::cout << net->getName() << std::endl;
                complete_state_machines_.insert(temp.begin(), temp.end());
//                std::cout << "complete_networks_ set " << std::endl;
//                for(auto net : complete_networks_)
//                    std::cout << net->getName() << std::endl;
//                std::cout << "complete net at :" << currentTimestamp << " from : "<<pattern.getName() <<" : \n";
//                for(auto net : complete_networks_)
//                    std::cout << "\t " <<net->getName() << std::endl;
            }

        }
        for (auto& pattern: actions_)
            pattern.cleanInvolve(set_valid_facts);
        flag_ = true;
    }
    return set_valid_facts;
}

void ActionMethod::displayCurrentState()
{
//    for(auto& pattern : patterns_)
//        for(auto& net : pattern.networks_) // TODO networks_ should not be public
//            std::cout << net->getCurrentState()->toString() << std::endl;
}

void ActionMethod::clean()
{
    flag_ = false;
    for (auto& pattern: actions_)
    {
        pattern.clean();
    }
    complete_state_machines_.clear();
}

void ActionMethod::cleanActions(std::set<uint32_t> set_id)
{
    for (auto& pattern: actions_)
    {
        pattern.cleanInvolve(set_id);
//        pattern.clean();
    }
}

std::string ActionMethod::toString()
{
    std::string res;
    for (auto& pattern: actions_)
        res += pattern.toString() + "\n";
    return res;
}

std::string ActionMethod::currentState(bool shortVersion)
{
    std::string res;
    for (auto& pattern: actions_)
        res += pattern.currentState(shortVersion) + "\n";
    return res;
}
bool ActionMethod::checkSubAction(ActionMethod* action)
{
    bool evolve = false;
    for (auto& pattern: actions_)
        evolve |= pattern.checksubAction(action);
    if (evolve)
        flag_ = false;
    return evolve;
}
bool ActionMethod::checkNewExplanation()
{
    bool res = false;
    if (flag_ == false)
    {
        for (auto& specializedAction: actions_)
            res |= specializedAction.checkNewUpdatedSubStateMachine();
    }

    return res;
}
std::vector<StateMachine*> ActionMethod::getNewExplanation()
{
    std::vector<StateMachine*> res;
    for (auto& specializedAction: actions_)
    {
        auto nets = specializedAction.getUpdatedStateMachines();
        res.insert(res.end(), nets.begin(), nets.end());
    }
    return res;
}
double ActionMethod::maxTtl()
{
    double res;
    for (auto& action: actions_)
        if (action.getTtl() > res)
            res = action.getTtl();
    return res;
}


} // namespace procedural