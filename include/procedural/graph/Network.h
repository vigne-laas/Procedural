#ifndef PROCEDURAL_NETWORK_H
#define PROCEDURAL_NETWORK_H

#include <unordered_set>
#include <unordered_map>
#include <map>
#include <vector>
#include <string>

#include "procedural/core/Graph/State.h"


namespace procedural {

class Network
{
public:
    Network(const std::vector<std::vector<FactPattern>>& patterns, const std::string& name, int id);
    Network(const std::string& name,int id);
    // Network(const Network& mother) = delete;

    void addTransitionIndex(const FactPattern& pattern, int32_t index);

    bool evolve(const Fact& fact);

    void displayNetwork();

    void displayVariables();

    void updateVariables(const Fact& fact, const Transition& update_transition);

    const State* getCurrentState() { return current_state_; }

    bool isComplete() { return current_state_->isFinalNode(); }


    Network* clone();

    std::string name_;
    uint32_t id_;

private:
    

    void buildNetwork(const std::vector<std::vector<FactPattern>>& patterns);

    void linkNetwork();

    void checkVar(const FactPattern& pattern);

//    std::unordered_map<std::string, int32_t> variables_map_;
    std::vector<Variable_t> variables_;
    std::unordered_set<std::string> literal_variables_;

    std::vector<State> graph_;
//    std::vector<Transition> initialTransition_;
    State* current_state_;

//    void init_graph(int32_t vector_size);

};

} // procedural

#endif //PROCEDURAL_NETWORK_H
