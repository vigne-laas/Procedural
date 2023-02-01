#ifndef PROCEDURAL_ACTION_H
#define PROCEDURAL_ACTION_H

#include <string>
#include <vector>
#include <map>

#include "procedural/core/Types/FactPattern.h"
#include "procedural/graph/Network.h"
#include "procedural/core/Types/PatternRecognition.h"

namespace procedural {

class Action
{
public:
    explicit Action(const std::string& name);

    void addPatterns(const PatternRecognition_t& pattern);

//    const std::vector<std::vector<FactPattern>>& getFacts();

    void feed(const Fact& fact);

    void addFacts(const FactPattern& facts);

    void checkCompleteNetworks();

    void displayCurrentState();

//    const std::vector<std::string>& getDescriptions();

//    const std::vector<std::string>& getVariables();

private:
    std::string name_;


//    void addDescriptions(Description des);



//    bool ordered_ = false;
    std::vector<uint32_t> flags;
    std::vector<Network*> root_networks_;
    std::vector<std::unordered_set<Network*>> networks_;
    std::unordered_set<Network*> complete_networks_;
    std::vector<PatternRecognition_t> patterns_;
//    std::vector<std::string> descriptions_;
};

} // namespace procedural

#endif // PROCEDURAL_ACTION_H
