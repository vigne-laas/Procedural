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

    bool addPatterns(const PatternRecognition & pattern);

    void feed(const Fact& fact);

    void checkCompleteNetworks();

    void displayCurrentState();

    std::string toString();

private:
    std::string name_;

    std::vector<PatternRecognition> patterns_;
};

} // namespace procedural

#endif // PROCEDURAL_ACTION_H
