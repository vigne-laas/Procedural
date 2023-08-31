#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>
#include <ontologenius/OntologyManipulator.h>

#include "procedural/core/Graph/Transitions/TransitionFact.h"
#include "procedural/core/Graph/Transitions/TransitionStateMachine.h"
#include "procedural/core/Graph/Transitions/TransitionActionMethod.h"
#include "procedural/core/Graph/Transitions/TransitionTask.h"

#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/Variable.h"



namespace procedural {
class Action;
class Task;

class State
{
public:
    explicit State(const std::string& name, int id);

    State* evolve(Fact* fact);
    std::pair<State*, TransitionStateMachine*> evolve(StateMachine* state_machine);
    State* evolve(Action* action);
    State* evolve(Task* task);

    void addTransition(const TransitionFact& transition, State* next_state);
    void addTransition(const TransitionStateMachine& transition, State* next_state);
    void addTransition(const TransitionActionMethod& transition, State* next_states);
    void addTransition(const TransitionTask& transition, State* next_states);

    void linkVariables(std::map<std::string, Variable_t>& variables_);

    void expandTransitions(onto::OntologyManipulator* onto_manipulator);
    bool isFinalNode() const { return nexts_facts_.empty() && nexts_state_machines_.empty() && nexts_actions_.empty() && nexts_tasks_.empty(); }
    int getId() const { return id_; };
    std::string toString() const;

    const std::vector<std::pair<TransitionFact, State*>> getNextsFacts() const { return nexts_facts_; };
    const std::vector<std::pair<TransitionStateMachine, State*>> getNextsStateMachines() const { return nexts_state_machines_; };
    const std::vector<std::pair<TransitionActionMethod, State*>> getNextsActions() const { return nexts_actions_; };
    const std::vector<std::pair<TransitionTask, State*>> getNextsTasks() const { return nexts_tasks_; };

    bool hasTimeoutTransition() const { return has_timeout_transition;}
    std::vector<State*> getParents_() {return parents_;};
    std::unordered_set<int> getConstrains_() {return valide_constrains_;};


    void addTimeoutTransition();

    void addParents(State* parent_state);

    void addValidateConstraints(const std::vector<int>& constrains);
    void addValidateConstraints(const std::unordered_set<int>& constrains);
    bool valideConstrains(const std::vector<int>& constrains);
    void closeTo(State* final_state, State* parent,State* origin);

private:
    int id_;
    std::string name_;
    bool initial_node_;

    std::vector<std::pair<TransitionFact, State*>> nexts_facts_;
    std::vector<std::pair<TransitionStateMachine, State*>> nexts_state_machines_;
    std::vector<std::pair<TransitionActionMethod, State*>> nexts_actions_;
    std::vector<std::pair<TransitionTask, State*>> nexts_tasks_;

    std::vector<State*> parents_;
    std::unordered_set<int> valide_constrains_;
    bool has_timeout_transition;
};

} // namespace procedural

#endif // PROCEDURAL_STATE_H
