#ifndef PROCEDURAL_PATTERNRECOGNITION_H
#define PROCEDURAL_PATTERNRECOGNITION_H

#include <vector>
#include "procedural/core/Types/PatternTransition.h"
#include "procedural/core/Graph/Network.h"
#include "procedural/core/Types/Fact.h"

namespace procedural {

class PatternRecognition
{
    public:
    PatternRecognition(const std::string& name_in,std::vector<procedural::PatternTransition_t>& patterns_in,std::vector<std::string>& descriptions_in);
    std::string name;
    std::vector<procedural::PatternTransition_t>  patterns;
    Network* root_Network; //issue when i try without *
    std::unordered_set<Network *> networks_;
    std::unordered_set<Network *> complete_networks;
    std::vector<std::string> descriptions;

    bool buildNetwork();

    int getNextId();
    void checkNetwork();

    void checkNetworkComplete(Network * net);
    
    void feed(const Fact& fact);


     
};

} // namespace procedural

#endif //PROCEDURAL_PATTERNRECOGNITION_H
