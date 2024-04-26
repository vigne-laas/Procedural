#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>
#include <set>
#include <string>
#include <vector>
#include <ontologenius/OntologyManipulator.h>
#include "iostream"

#include "procedural/action_recognition/core/internal_structures/state_machine/structures/StateEvolveResult.h"
#include "procedural/action_recognition/core/internal_structures/state_machine/structures/ActionObservation.h"
#include "procedural/action_recognition/core/internal_structures/state_machine/ActionTransition.h"
#include "procedural/utils/structures/Variable.h"

namespace procedural {
using namespace recognition;

class State {
public:
    explicit State(const std::string& name, int id);

    StateEvolveResult_t* evolve(ActionObservation_t* obs);

     template <typename PatternType, ActionTransitionType transitionType>
     void addTransition(const ActionTransition<PatternType, transitionType>& transition, State* next_state)
     {
         childreens.emplace_back(transition, next_state);
     }

    void close(std::map<std::string, Variable_t>& variables_);

    void expandTransitions(onto::OntologyManipulator* onto_manipulator);

    bool isFinalNode() const
    {
        return final_node_;
    }

    int getId() const { return id_; };

    std::string toString() const;

    std::string toShortString() const;

    const std::vector<std::pair<ActionTransition_t, State*>> getNextState() const { return childreens; };

    void set_new_id(int new_id) { id_ = new_id; };

    std::set<State*> getParents_() { return parents_; }

    void addParents(State* parent_state) { parents_.insert(parent_state); };


    std::string getFullName() { return name_ + "_" + std::to_string(id_); };

    // Save the DOT file
    void saveDOTFile(std::ofstream& dot_file) const;

    const int& getLevel() const { return level_; };

private:

    // Generate DOT specific to transitions of type Fact
    void generateDOT_Transition(std::ofstream& dotFile, std::set<int>& visitedStates) const;


    int id_;
    std::string name_;
    bool initial_node_;
    bool final_node_;

    std::vector<std::pair<ActionTransition_t, State*>> childreens;

    std::set<State*> parents_;
    int level_;

};

} // procedural

#endif //PROCEDURAL_STATE_H
