#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>
#include "procedural/core/Graph/Transition.h"
#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/Variable.h"


namespace procedural {

class Transition;

class Fact;

class State
{
public:
    explicit State(const std::string& name, int id);

    State* evolve(const Fact& fact) const;

    void addTransition(const Transition& transition, State* pnext_state);

//    void setTransition(const std::vector<Transition>& transitions);

    void setInitialNode()
    { initial_node_ = true; }

    bool isInitialNode() const { return initial_node_; }
    bool isFinalNode() const { return nexts_.empty(); }

//    std::vector<Transition>& getTransitions();
    void link_transitions(std::vector<Variable_t>& variables_);

    std::string toString();

    void expand_transitions();

private:
    std::string name_;
    bool initial_node_;
    std::vector<std::pair<Transition, State*>> nexts_;
};

} // procedural

#endif //PROCEDURAL_STATE_H
