#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>

#include "procedural/core/Graph/TransitionFact.h"
#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/Variable.h"
#include "procedural/core/Graph/TransitionStateMachine.h"

namespace procedural {

class State
{
public:
    explicit State(const std::string& name, int id);

    State* evolve(Fact* fact);
    std::pair<State*, TransitionStateMachine*> evolve(StateMachine* state_machine);

    void addTransition(const TransitionFact& transition, State* next_state);
    void addTransition(const TransitionStateMachine& transition, State* next_state);
    void linkVariables(std::map<std::string, Variable_t>& variables_);

    void expandTransitions(ObjectPropertyClient* object_client);
    bool isFinalNode() const { return nexts_facts_.empty() && nexts_state_machines_.empty(); }
    uint32_t getId() const { return id_; };
    std::string toString() const;

    const std::vector<std::pair<TransitionFact, State*>> getNextsFacts() const { return nexts_facts_; };
    const std::vector<std::pair<TransitionStateMachine, State*>> getNextsStateMachines() const { return nexts_state_machines_; };
    bool hasTimeoutTransition() const { return has_timeout_transition;}


    void addTimeoutTransition();
private:
    uint32_t id_;
    std::string name_;
    bool initial_node_;
    std::vector<std::pair<TransitionFact, State*>> nexts_facts_;
    std::vector<std::pair<TransitionStateMachine, State*>> nexts_state_machines_;
    bool has_timeout_transition;
};

} // namespace procedural

#endif // PROCEDURAL_STATE_H
