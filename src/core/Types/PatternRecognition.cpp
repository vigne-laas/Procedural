#include "procedural/core/Types/PatternRecognition.h"

#include <iostream>

namespace procedural
{
    PatternRecognition::PatternRecognition(const std::string& name_in,std::vector<procedural::PatternTransition_t>& patterns_in,std::vector<std::string>& descriptions_in):patterns(patterns_in),name(name_in),descriptions(descriptions_in)
    {
    
    }

    bool PatternRecognition::buildNetwork()
    {
        root_Network = new Network(name,0);
        for(auto&  pattern : patterns)
            root_Network->addTransition(pattern);
        return root_Network->closeNetwork();
    }

    int PatternRecognition::getNextId()
    {
        return 10;
    }

    void PatternRecognition::checkNetwork()
    {
        for(auto& net : networks_)
            checkNetworkComplete(net);
    }
    void PatternRecognition::checkNetworkComplete(Network * net)
    {
        if(net->isComplete())
        {
            complete_networks.insert(net);
            networks_.erase(net);
        }
    }

    void PatternRecognition::feed(const Fact& fact)
    {
        bool evolve = false;
        for(auto& net : networks_)
        {
            if(net->evolve(fact));
            {
                std::cout << "evolve of " << net->getName() << std::endl;
                evolve = true;
                // checkNetworkComplete(net);
            }
        }
        if(not evolve)
        {
            std::cout << "create new network "<< std::endl;
            Network* new_net = root_Network->clone(getNextId());
            if(new_net->evolve(fact))
            {
                networks_.insert(new_net);
                // checkNetworkComplete(new_net);
            }                
        }

    }
    
    
} //procedural