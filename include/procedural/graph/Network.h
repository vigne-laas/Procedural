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

    const State* getCurrentState() { return current_state_; }

    bool isComplete() { return current_state_->isFinalNode(); }

    void addTransition(const PatternTransition_t& pattern);
    void addState(int id_state);
    std::string map2String(); // change name

    Network* clone();

    void displayVariables();

private:
    void linkNetwork();

    void insertVariable(const std::string& variable);

    bool processInitialState();

    std::string name_;
    uint32_t id_;

    // TODO check if map could be used
    std::vector<Variable_t> variables_;
    std::unordered_set<std::string> literal_variables_;

    State* current_state_;
    std::map<int, State*> states_;
    int id_initial_state_;
};

} // procedural

#endif //PROCEDURAL_NETWORK_H
