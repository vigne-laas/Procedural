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
    PatternRecognition(const std::string& name,
                      std::vector<procedural::PatternTransition_t>& patterns,
                      std::vector<std::string>& descriptions);

    bool isValid() const { return is_valid_; }
    int getNextId();
    void checkNetwork();

    void checkNetworkComplete(Network * net);
    
    void feed(const Fact& fact);

    std::string toString();
private:
    static std::unordered_set<int> set_id;

    std::string name_;
    Network* root_network_; //issue when i try without *
    std::unordered_set<Network *> networks_;
    std::unordered_set<Network *> complete_networks_;
    std::vector<std::string> descriptions_;

    bool is_valid_;
};

} // namespace procedural

#endif //PROCEDURAL_PATTERNRECOGNITION_H
