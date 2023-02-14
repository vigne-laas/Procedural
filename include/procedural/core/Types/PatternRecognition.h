#ifndef PROCEDURAL_PATTERNRECOGNITION_H
#define PROCEDURAL_PATTERNRECOGNITION_H

#include <vector>
#include "procedural/core/Types/PatternTransition.h"
#include "procedural/core/Graph/Network.h"
#include "procedural/core/Types/Fact.h"

namespace procedural {

struct PatternRecognition_t
{
    
    PatternRecognition_t(const std::string& name_in,std::vector<procedural::PatternTransition_t>& patterns_in,std::vector<std::string>& descriptions_in):patterns(patterns_in),name(name_in),descriptions(descriptions_in)
    {    
    };
    std::string name;
    std::vector<procedural::PatternTransition_t>  patterns;
    Network* root_Network; //issue when i try without *
    std::unordered_set<Network *> networks_;
    std::unordered_set<Network *> complete_networks;
    std::vector<std::string> descriptions;

    bool buildNetwork()
    {
        root_Network = new Network(name,0);
        for(auto&  pattern : patterns)
            root_Network->addTransition(pattern);
        return root_Network->closeNetwork();
    }

    int getNextId()
    {   
        // random + check on onto if already done 
        return 10;
    }
    void checkNetwork()
    {
        for(auto& net : networks_)
        {
            checkNetworkComplete(net);
        }
    }

    void checkNetworkComplete(Network * net)
    {
        if(net->isComplete())
        {
            complete_networks.insert(net);
            networks_.erase(net);
        }
    }

    void feed(const Fact& fact)
    {
        bool evolve = false;
        for(auto& net : networks_)
        {
            if(net->evolve(fact));
            {
                evolve = true;
                // checkNetworkComplete(net);
            }
        }
        if(not evolve)
        {
            Network* new_net = root_Network->clone(getNextId());
            if(new_net->evolve(fact))
            {
                networks_.insert(new_net);
                // checkNetworkComplete(new_net);
            }                
        }

    }


     
};

} // namespace procedural

#endif //PROCEDURAL_PATTERNRECOGNITION_H
