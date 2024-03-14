#ifndef PROCEDURAL_STATE_H
#define PROCEDURAL_STATE_H

#include <unordered_set>
#include <ontologenius/OntologyManipulator.h>

#include "procedural/old/core/Graph/Transitions/TransitionFact.h"
#include "procedural/old/core/Graph/Transitions/TransitionAction.h"
#include "procedural/old/core/Graph/Transitions/TransitionActionMethod.h"
#include "procedural/old/core/Graph/Transitions/TransitionTask.h"

#include "procedural/old/core/Types/Fact.h"
#include "procedural/old/core/Types/Variable.h"


namespace procedural {
class Action;
class Task;
class State;

struct Transitions_t
{
//    std::vector<std::pair<TransitionFact, State*>> next_facts_;
    std::set<TransitionAction> next_actions_;
//    std::vector<std::pair<TransitionActionMethod, State*>> next_actions_methods_;
    std::set<TransitionTask> next_tasks_;

    std::string toString()
    {
        std::string msg;
        if (!next_actions_.empty())
        {
            msg += "\t actions T: ";
            for (auto it = next_actions_.begin(); it != next_actions_.end(); it++)
            {
                msg += it->toShortString();
                if (std::next(it) != next_actions_.end())
                    msg += ", ";

            }
        }

        if (!next_tasks_.empty())
        {
            msg += "\t tasks T: ";
            for (auto it = next_tasks_.begin(); it != next_tasks_.end(); it++)
            {
                msg += it->toShortString();
                if (std::next(it) != next_tasks_.end())
                    msg += ", ";

            }
        }
        return msg;
    };


};

class State
{
public:
    explicit State(const std::string& name, int id);

    State* evolve(Fact* fact);
    std::pair<State*, TransitionAction*> evolve(StateMachine* state_machine);
    State* evolve(Action* action);
    State* evolve(Task* task);

    void addTransition(const TransitionFact& transition, State* next_state);
    void addTransition(const TransitionAction& transition, State* next_state);
//    void addTransition(const TransitionActionMethod& transition, State* next_state);
    void addTransition(const TransitionTask& transition, State* next_state);

    void linkVariables(std::map<std::string, Variable_t>& variables_);

    void expandTransitions(onto::OntologyManipulator* onto_manipulator);
    bool isFinalNode() const
    {
        return next_facts_.empty() && next_actions_.empty() && next_tasks_.empty();
    }
    int getId() const { return id_; };
    std::string toString() const;
    std::string toShortString() const;

    const std::vector<std::pair<TransitionFact, State*>> getNextFacts() const { return next_facts_; };
    const std::vector<std::pair<TransitionAction, State*>> getNextStateMachines() const { return next_actions_; };
//    const std::vector<std::pair<TransitionActionMethod, State*>>
//    getNextActions() const { return next_actions_methods_; };
    const std::vector<std::pair<TransitionTask, State*>> getNextTasks() const { return next_tasks_; };

    void set_new_id(int new_id) { id_ = new_id; };

    bool hasTimeoutTransition() const { return has_timeout_transition; }
    std::set<State*> getParents_() { return parents_; }
    std::unordered_set<int> getConstrains_() { return valide_constrains_; }


    void addTimeoutTransition(State* final_state);

    void addParents(State* parent_state);

    void addValidateConstraints(const std::vector<int>& constrains);
    void addValidateConstraints(const std::unordered_set<int>& constrains);
    void addValidateConstraints(int constrain);
    bool validateConstraints(const std::vector<int>& constrains);
    bool validateConstraints(const std::unordered_set<int>& constraints) const;
    void closeTo(State* final_state, State* parent, State* origin);
    void closeTo(std::vector<State*> possible_states, Transitions_t transitions);

    std::map<State*, Transitions_t> getValideParents(std::vector<int> constraints);
    std::map<State*, Transitions_t>
    getValideParents(std::vector<int> constraints, std::map<State*, Transitions_t>& map, State* origin_state);

    State* doTimeoutTransition();

    std::string getFullName() { return name_ + "_" + std::to_string(id_); };


    // Save the DOT file
    void saveDOTFile(std::ofstream& dot_file) const;

    const int& getLevel() const { return level_; };
private:

    // Generate DOT specific to transitions of type Fact
    void generateDOT_Facts(std::ofstream& dotFile, std::set<int>& visitedStates) const;

    // Generate DOT specific to transitions of type Action
    void generateDOT_Actions(std::ofstream& dotFile, std::set<int>& visitedStates) const;

    // Generate DOT specific to transitions of type Task
    void generateDOT_Tasks(std::ofstream& dotFile, std::set<int>& visitedStates) const;
    int id_;
    std::string name_;
    bool initial_node_;

    std::vector<std::pair<TransitionFact, State*>> next_facts_;
    std::vector<std::pair<TransitionAction, State*>> next_actions_;
//    std::vector<std::pair<TransitionActionMethod, State*>> next_actions_methods_;
    std::vector<std::pair<TransitionTask, State*>> next_tasks_;

    std::set<State*> parents_;
    std::unordered_set<int> valide_constrains_;
    bool has_timeout_transition;
    State* final_state_;
    int level_;
};

} // namespace procedural

#endif // PROCEDURAL_STATE_H
