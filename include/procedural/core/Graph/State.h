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

    State* evolve(Fact* fact);

    void addTransition(const Transition& transition, State* next_state);
    void linkVariables(std::map<std::string, Variable_t>& variables_);


    void expandTransitions();
    bool isFinalNode() const{ return nexts_.empty(); }
    uint32_t getId(){ return id_; };
    std::string toString() const;

    const std::vector<std::pair<Transition, State*>> getNexts(){ return nexts_; };


private:
    uint32_t id_;
    std::string name_;
    bool initial_node_;
    std::vector<std::pair<Transition, State*>> nexts_;
};

} // namespace procedural

#endif // PROCEDURAL_STATE_H
