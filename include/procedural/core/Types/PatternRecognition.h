#ifndef PROCEDURAL_PATTERNRECOGNITION_H
#define PROCEDURAL_PATTERNRECOGNITION_H

#include <vector>
#include "procedural/core/Types/PatternTransition.h"
#include "procedural/core/Graph/Network.h"

#include "procedural/core/Types/Fact.h"


namespace procedural {
class Action;
class PatternRecognition
{
public:
    PatternRecognition(const std::string& name,
                       const std::vector<PatternTransition_t>& patterns,
                       const std::vector<PatternNetworkTransition_t>& patterns_network,
                       const std::vector<ActionDescription_t>& descriptions,
                       uint32_t ttl);

    bool isValid() const { return is_valid_; }
    int getNextId();
    std::set<uint32_t> checkNetwork();

    void cleanInvolve(const std::set<uint32_t>& list_valid_facts);
    void clean();

    std::unordered_set<Network*> getCompleteNetwork() { return complete_networks_;};

    void feed(Fact* fact);

    std::string toString();

    std::string currentState(bool shortVersion = true);

    bool checksubAction(Action* action);
private:
    int id_;

    std::string name_;
    Network* root_network_; //issue when i try without *

    std::unordered_set<Network*> networks_;
    std::unordered_set<Network*> complete_networks_;
//    std::unordered_set<Network*> networks_to_del_;

    bool is_valid_;
    bool evolve_sub_action;
    uint32_t time_to_live_;
};

} // namespace procedural

#endif //PROCEDURAL_PATTERNRECOGNITION_H
