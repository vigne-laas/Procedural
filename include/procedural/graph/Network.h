#ifndef PROCEDURAL_NETWORK_H
#define PROCEDURAL_NETWORK_H

#include <unordered_set>
#include <unordered_map>
#include <map>
#include <vector>
#include <string>

#include "procedural/core/Graph/State.h"
#include "procedural/core/Types/PatternTransition.h"


namespace procedural {

class Network
{
public:
    Network(const std::string& name,int id);
    Network(const Network& other) = delete;

    bool evolve(const Fact& fact);

    void updateVariables(const Fact& fact, const Transition& update_transition);

    const State* getCurrentState() { return current_state_; }

    bool isComplete() { return current_state_->isFinalNode(); }

    void addState(const PatternTransition_t& pattern);
    void checkState(int id_state);
    std::string map2String();

    Network* clone();

    void displayNetwork();
    void displayVariables();

    std::string name_;
    uint32_t id_;

private:
    void linkNetwork();

    void checkVar(const FactPattern& pattern);

    std::vector<Variable_t> variables_;
    std::unordered_set<std::string> literal_variables_;

    std::vector<State> graph_;
    State* current_state_;
    std::map<int, State*> states_;
    int id_initial_state_;
};

} // procedural

#endif //PROCEDURAL_NETWORK_H
