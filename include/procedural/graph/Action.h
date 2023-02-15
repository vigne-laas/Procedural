#ifndef PROCEDURAL_ACTION_H
#define PROCEDURAL_ACTION_H

#include <string>
#include <vector>
#include <map>

#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/PatternRecognition.h"

namespace procedural {


class Action
{
public:
    explicit Action(const std::string& name);

    

    void addPatterns(PatternRecognition & pattern);
    void close();

//    const std::vector<std::vector<FactPattern>>& getFacts();

    void feed(const Fact& fact);

    void checkCompleteNetworks();

    void displayCurrentState();

    std::string toString();

//    const std::vector<std::string>& getDescriptions();

//    const std::vector<std::string>& getVariables();

private:
    std::string name_;


//    void addDescriptions(Description des);



//    bool ordered_ = false;
    // std::vector<uint32_t> flags;
    std::vector<PatternRecognition> patterns_;
//    std::vector<std::string> descriptions_;
};

} // namespace procedural

#endif // PROCEDURAL_ACTION_H
