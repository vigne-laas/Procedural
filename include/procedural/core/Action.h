#ifndef PROCEDURAL_ACTION_H
#define PROCEDURAL_ACTION_H

#include <string>
#include <vector>
#include <map>
#include "procedural/core/FactPattern.h"
#include "procedural/graph/Network.h"


namespace procedural {

class Action
{
public:
    Action(const std::string& name);

    const std::vector<std::vector<FactPattern>>& getFacts();

    void feed(const Fact& fact);

    void addFacts(const FactPattern& facts);
    void checkCompleteNetworks();

//    const std::vector<std::string>& getDescriptions();

//    const std::vector<std::string>& getVariables();

private:
    std::string name_;


//    void addDescriptions(Description des);



//    bool ordered_ = false;
//    std::vector<std::string> variables_;
    std::vector<Network> networks_;
    std::vector<Network> complete_networks_;
    std::vector<std::vector<FactPattern>> facts_;
    std::vector<std::string> descriptions_;
};

} // procedural

#endif //PROCEDURAL_ACTION_H
