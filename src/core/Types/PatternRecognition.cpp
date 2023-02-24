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

void PatternRecognition::checkNetwork()
{
    for(auto net : networks_)
        checkNetworkComplete(net);
        
    for(auto& net : complete_networks_)
    {
        std::cout << "network finish :" << net->getName() << std::endl;
        std::cout << "explanation : " << net->explain() << std::endl;
        networks_.erase(net);
        for(auto& msg : descriptions_)
            std::cout << "\t" << msg << std::endl;
    }
}

void PatternRecognition::checkNetworkComplete(Network * net)
{
    if(net->isComplete())
        complete_networks_.insert(net);
}

void PatternRecognition::feed(Fact* fact)
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