#include "procedural/core/Types/PatternRecognition.h"

#include <cmath>
#include <iostream>

namespace procedural
{

int PatternRecognition::id_ = 0;

PatternRecognition::PatternRecognition(const std::string& name,
                                        std::vector<procedural::PatternTransition_t>& patterns,
                                        std::vector<ActionDescription_t>& descriptions) : name_(name),is_valid_(false)
{
    root_network_ = new Network(name_, 0);
    for(auto& pattern : patterns)
        root_network_->addTransition(pattern);
    for(auto& des : descriptions)
        root_network_->addDescription(des);
    is_valid_ = root_network_->closeNetwork();
}

int PatternRecognition::getNextId()
{
    return id_++;
}

std::vector<std::vector<uint32_t>> PatternRecognition::checkNetwork()
{
    std::vector<std::vector<uint32_t>> list_valid_facts;
    for(auto net : networks_)
        checkNetworkComplete(net);
        
    for(auto& net : complete_networks_)
    {
        std::cout << "network finish :" << net->getName() << std::endl;
        std::cout << "explanation : " << net->describe() << std::endl;
        std::cout << "facts involved : ";
        for(auto& id : net->getIdsFacts())
            std::cout << std::to_string(id) << "|"; 
        list_valid_facts.push_back(net->getIdsFacts());
        std::cout << std::endl;
        networks_.erase(net);
    }
    std::vector<Network*> network_to_deletes;
    // std::cout << "Pattern : "<< this->name_<< std::endl;
    // std::cout << "size net : "<< networks_.size()<< std::endl;
    for(auto& list : list_valid_facts)
    {
        for(auto& net: networks_)
        {
            if(net->involveFacts(list))
            {
                std::cout << "may delete this network" << net->getName() << std::endl;
                network_to_deletes.push_back(net);
            }
        }
    }
    for(auto net : network_to_deletes)
    {
        // delete net;
        networks_.erase(net);
    }

    return list_valid_facts;
}

void PatternRecognition::checkNetworkComplete(Network * net)
{
    if(net->isComplete())
        complete_networks_.insert(net);
}

void PatternRecognition::cleanInvolve(const std::vector<uint32_t>& list_valid_facts)
{
    std::vector<Network*> network_to_deletes;

    for(auto& net: networks_)
    {
        if(net->involveFacts(list_valid_facts))
        {
            std::cout << "may delete this network" << net->getName() << std::endl;
            network_to_deletes.push_back(net);
        }
    }
    for(auto net : network_to_deletes)
    {
        // delete net;
        networks_.erase(net);
    }


}
void PatternRecognition::feed(Fact *fact)
{
    bool evolve = false;
    for(auto& net : networks_)
    {   
        if(net->evolve(fact))
        {
            // std::cout << "\t succes of evolution" << std::endl;
            evolve = true;
            // checkNetworkComplete(net);
        }
    }

    if(evolve == false)
    {
        std::cout << "create new network of " << name_ << std::endl;
        Network* new_net = root_network_->clone(getNextId());
        if(new_net->evolve(fact))
        {
            networks_.insert(new_net);
            // checkNetworkComplete(new_net);
        }
        else
            delete new_net;              
    }
}

std::string PatternRecognition::toString()
{
    std::string res;
    res += "Pattern Recognition of : " + name_ + "\n";
    res += "\t nb of active networks : " +std::to_string(networks_.size())+"\n";
    res += "\t active networks : ";
    for(auto& net : networks_)
        res+= net->toString()+"\n";
    return res;
}
    
} // namespace procedural