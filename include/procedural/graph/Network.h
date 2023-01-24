#ifndef PROCEDURAL_NETWORK_H
#define PROCEDURAL_NETWORK_H

#include <unordered_set>
#include <unordered_map>
#include <map>
#include <vector>
#include <string>

#include "procedural/graph/Transition.h"
#include "procedural/graph/State.h"
#include "procedural/core/Types/FactPattern.h"
#include "procedural/core/Types/Fact.h"




namespace procedural {

//class FactPattern;
//class Transition;
//class State;

class Network
{
    friend Transition;
public:
    Network(const std::vector<std::vector<FactPattern>>& patterns, const std::string& name);

    void addTransitionIndex(const FactPattern& pattern, int32_t index);

    bool evolve(const Fact& fact);

    void displayNetwork();
    void displayVariables();

    void updateVariables(const Fact& fact, const Transition& update_transition);


private:

    std::unordered_map<std::string, int32_t> variables_map_;
    std::string name_;
    uint32_t id_;
    std::vector<State> graph_;
    std::vector<Transition> initialTransition_;
    State* current_state_;

//    void init_graph(int32_t vector_size);
};

} // procedural

#endif //PROCEDURAL_NETWORK_H
