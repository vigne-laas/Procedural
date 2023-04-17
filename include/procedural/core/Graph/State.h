#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>

#include "procedural/core/Graph/TransitionFact.h"
#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/Variable.h"
#include "procedural/core/Graph/TransitionNetwork.h"

namespace procedural {

class State
{
public:
    explicit State(const std::string& name, int id);

    State* evolve(Fact* fact);
    State* evolve(Network* network);

    void addTransition(const TransitionFact& transition, State* next_state);
    void addTransition(const TransitionNetwork& transition, State* next_state);
    void linkVariables(std::map<std::string, Variable_t>& variables_);

    void expandTransitions();
    bool isFinalNode() const { return nexts_facts_.empty() && nexts_networks_.empty(); }
    uint32_t getId() const { return id_; };
    std::string toString() const;

    const std::vector<std::pair<TransitionFact, State*>> getNextsFacts() const { return nexts_facts_; };
    const std::vector<std::pair<TransitionNetwork, State*>> getNextsNetworks() const { return nexts_networks_; };

private:
    uint32_t id_;
    std::string name_;
    bool initial_node_;
    std::vector<std::pair<TransitionFact, State*>> nexts_facts_;
    std::vector<std::pair<TransitionNetwork, State*>> nexts_networks_;
};

} // namespace procedural

#endif // PROCEDURAL_STATE_H
