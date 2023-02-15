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
    Network(const std::string& name, int id);
    Network(const Network& other) = delete;

    bool evolve(const Fact& fact);

    const State* getCurrentState() { return current_state_; }
    std::string getName() { return full_name_; }

    bool isComplete() { return current_state_->isFinalNode(); }
    bool isClosed() { return closed_; }
    bool isValid() { return valid_; }

    bool addTransition(const PatternTransition_t& pattern);
    void addState(int id_state);
    bool closeNetwork();

    std::string toString(); 

    Network* clone(int new_id);
    void displayVariables();
    std::string explain();

private:
    void linkNetwork();

    void insertVariable(const std::string& variable);

    bool processInitialState();


    std::string name_;
    uint32_t id_;
    std::string full_name_;

    std::map<std::string,Variable_t> variables_;

    State* current_state_;
    std::map<int, State*> states_;
    int id_initial_state_;

    std::vector<Fact> list_facts_valid;

    bool closed_;
    bool valid_;
};

} // procedural

#endif //PROCEDURAL_NETWORK_H
