#include "procedural/core/Types/PatternRecognition.h"

#include <cmath>
#include <iostream>
#include <set>

namespace procedural {


PatternRecognition::PatternRecognition(const std::string& name,
                                       const std::vector<procedural::PatternTransition_t>& patterns,
                                       const std::vector<ActionDescription_t>& descriptions,
                                       uint32_t ttl) : name_(name),
                                                       is_valid_(false),
                                                       time_to_live_(ttl),
                                                       id_(0)
{
    root_network_ = new Network(name_, 0);
    for (auto& pattern: patterns)
        root_network_->addTransition(pattern);
    for (auto& des: descriptions)
        root_network_->addDescription(des);
    is_valid_ = root_network_->closeNetwork();
}

int PatternRecognition::getNextId()
{
    return id_++;
}

std::set<uint32_t> PatternRecognition::checkNetwork()
{
    std::unordered_set<Network*> complete_networks;
    std::unordered_set<Network*> networks_to_del;

    std::set<uint32_t> set_valid_facts;
    for (auto net: networks_)
    {
        if (net->isComplete())
            complete_networks.insert(net);
        if (net->getAge() > time_to_live_)
        {
            networks_to_del.insert(net);
            std::cout << "Del net due to age " << net->getName() << std::endl;
        }
    }

    for (auto& net: complete_networks)
    {
        std::cout << "network finish :" << net->getName() << std::endl;
        std::cout << "explanation : " << net->describe() << std::endl;
        std::cout << "facts involved : ";
        for (auto& id: net->getIdsFacts())
        {
            std::cout << std::to_string(id) << "|";
            set_valid_facts.insert(id);
        }
        std::cout << std::endl;
        networks_.erase(net);
        delete net; // Check if Guillaume is right
    }
    // std::cout << "Pattern : "<< this->name_<< std::endl;
    // std::cout << "size net : "<< networks_.size()<< std::endl;
    for (auto& net: networks_)
    {
        if (net->involveFacts(set_valid_facts))
        {
            std::cout << "may delete this network" << net->getName() << std::endl;
            networks_to_del.insert(net);
        }
    }

    for (auto net: networks_to_del)
    {
        networks_.erase(net);
        delete net;
    }

    return set_valid_facts;
}

void PatternRecognition::cleanInvolve(const std::set<uint32_t>& list_valid_facts)
{
    std::vector<Network*> network_to_deletes;

    for (auto& net: networks_)
    {
        if (net->involveFacts(list_valid_facts))
        {
//            std::cout << "may delete this network" << net->getName() << std::endl;
            network_to_deletes.push_back(net);
        }
    }

    for (auto net: network_to_deletes)
    {
        std::cout << "delete network  : " << net->getName() << std::endl;
        networks_.erase(net);
        delete net;
    }
}

void PatternRecognition::feed(Fact* fact)
{
    bool evolve = false;
    for (auto& net: networks_)
    {
        if (net->evolve(fact))
        {
            std::cout << "\t succes of evolution  : " << net->getName() << std::endl;
            evolve = true;
            // checkNetworkComplete(net);
        }
    }

    if (evolve == false)
    {
        Network* new_net = root_network_->clone(getNextId());
        if (new_net->evolve(fact))
        {
            std::cout << "create new network " << new_net->getName() << std::endl;
            networks_.insert(new_net);
            // checkNetworkComplete(new_net);
        } else
            delete new_net;
    }
}

std::string PatternRecognition::toString()
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\n";
    res += "\t nb of active networks : " + std::to_string(networks_.size()) + "\n";
    res += "\t active networks : \n";
    for (auto& net: networks_)
        res += net->toString() + "\n";
    return res;
}

std::string PatternRecognition::currentState(bool shortVersion)
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\t";
    res += "\t nb of networks : " + std::to_string(networks_.size()) + "\n";
    for (auto& net: networks_)
    {
        if (shortVersion)
            res += net->getCurrentState()->toString() + "\t";
        else
            res += net->toString() + "\n";
    }

    res += "\n";
    return res;
}

} // namespace procedural