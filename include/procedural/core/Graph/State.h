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

    State* evolve(const Fact& fact);

    void addTransition(const Transition& transition, State* next_state);

    void setInitialNode() { initial_node_ = true; }

    bool isInitialNode() const { return initial_node_; }
    bool isFinalNode() const { return nexts_.empty(); }

    void linkTransitions(std::vector<Variable_t>& variables_);
    void expandTransitions();

    std::string toString();

private:
    std::string name_;
    bool initial_node_;
    std::vector<std::pair<Transition, State*>> nexts_;
};

} // namespace procedural

#endif // PROCEDURAL_STATE_H
