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
        int iter = 1;
        std::srand(time(NULL));
        int id = std::rand() % (int)(std::pow(10, iter));
        
        while(set_id.find(id)!=set_id.end())
        {
            iter++;
            id = std::rand() % (int)std::pow(10, iter);
        }
        set_id.insert(id);
        // std::cout << "final id generated : " + std::to_string(id) << std::endl;
        // std::cout << "id already done : " << std::endl;
        // for(auto id : set_id)
            // std::cout << "\t " << std::to_string(id) << std::endl;
        return id;
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

    std::string PatternRecognition::toString()
    {
        std::string res;
        res += "Pattern Recognition of : " +name+"\n";
        res += "\t nb of active networks : " +std::to_string(networks_.size())+"\n";
        res += "\t active networks : ";
        for(auto& net : networks_)
            res+= net->toString()+"\n";
        // std::cout<<res<<std::endl;
        return res;
    }
    
    
} //procedural