#include <iostream>
#include "procedural/graph/Action.h"

namespace procedural {

Action::Action(const std::string& name) : name_(name)
{

}

void Action::feed(const Fact& fact)
{
    std::unordered_set<Network*> set_to_erase;
    for (auto it_set_network = networks_.begin(); it_set_network < networks_.end(); it_set_network++)
    {

        for (auto network: *it_set_network)
        {
            if (network->evolve(fact))
            {
//                std::cout <<"index : " << std::distance(networks_.begin(), it_set_network) <<std::endl;
                flags[std::distance(networks_.begin(), it_set_network)] = 1;
            }


//            network->displayNetwork();
            if (network->isComplete())
            {
                complete_networks_.insert(network);
                set_to_erase.insert(network);
            }
        }

        for (auto network_to_erase: set_to_erase)
            (*it_set_network).erase(network_to_erase);
    }

    for (auto i = 0; i < flags.size(); i++)
    {
        if (not flags[i])
        {

            // Network* newNetwork = root_networks_[i]->clone();
            // std::cout << "size "<< networks_[i].size() << std::endl;
            Network * newNetwork = new Network(patterns_[i].patterns,name_+std::to_string(networks_.size()+1),networks_[i].size()+1);
            std::cout << "try create new network : " << i <<" -> " <<fact.toString()<< std::endl;
            
            if (newNetwork->evolve(fact))
            {
                std::cout << "create network " << std::endl;
//                if(networks_[i])
                networks_[i].insert(newNetwork);
                // std::cout << newNetwork->getCurrentState()->toString() << std::endl;;
            }
            else
                delete newNetwork;
        }

    }


    checkCompleteNetworks();
}

void Action::checkCompleteNetworks()
{
    for (auto complete_network: complete_networks_)
    {
        std::cout << "-----------------------------" << std::endl;
        std::cout << "finish network " << complete_network->name_ << "_" << complete_network->id_ << std::endl;
        complete_network->displayVariables();
    }
}

void Action::displayCurrentState()
{
    for(auto network : networks_)
    {
        for(auto net : network)
        {
            std::cout << net->getCurrentState()->toString() << std::endl;
        }
    }

}
// const std::vector<std::vector<FactPattern>>& Action::getFacts()
//{
//     return facts_;
// }

void Action::addFacts(const FactPattern& facts)
{

}

void Action::addPatterns(const PatternRecognition_t& pattern)
{
    patterns_.push_back(pattern);
    Network* N = new Network(pattern.patterns,name_, 0);
//    N->displayNetwork();
    root_networks_.push_back(N);
//    for(auto net: root_networks_)
//        net->displayNetwork();
    flags.resize(root_networks_.size(), 0);
    networks_.resize(root_networks_.size(),{});
}

} // namespace procedural