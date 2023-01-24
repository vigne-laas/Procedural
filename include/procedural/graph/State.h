#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>
#include "procedural/graph/Transition.h"
#include "procedural/core/Fact.h"


namespace procedural {

class Transition;
class Fact;

class State
{
public:
    State(std::string name);

    State* evolve(const Fact& fact) const;

    void addTransition(const Transition& transition);

    void setTransition(const std::vector<Transition>& transitions);

    void setInitialNode();
    void setFinalNode();

    bool isInitialNode() const;
    bool isFinalNode() const;

    std::vector<Transition>& getTransitions();

    std::string toString();
    int id_;

    std::string name_;
private:

    bool final_node_;
    bool initial_node_;
    std::vector<Transition> transitions_;

};

} // procedural

#endif //PROCEDURAL_STATE_H
