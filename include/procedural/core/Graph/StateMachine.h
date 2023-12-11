#ifndef PROCEDURAL_STATE_MACHINE_H
#define PROCEDURAL_STATE_MACHINE_H

#include <unordered_set>
#include <unordered_map>
#include <map>
#include <exception>
#include <vector>
#include <string>
#include <set>

#include "procedural/core/Graph/State.h"
#include "procedural/core/Types/PatternTransitionFact.h"
#include "procedural/core/Types/Description.h"
#include "procedural/core/Types/ActionDescription.h"
#include "procedural/core/Types/PatternTransitionStateMachine.h"
#include "procedural/core/Types/WordTable.h"
#include "procedural/core/Types/IncompleteStateMachine.h"
#include "procedural/core/Types/Interface.h"
#include "procedural/core/Types/ResultFeedProcess.h"
#include "procedural/core/Types/HTNTransition.h"

namespace procedural {

struct StateMachineException : public std::exception
{
    std::string msg_;
    explicit StateMachineException(const std::string& msg) : msg_(msg) {}
    const char* what() const throw()
    {
        return msg_.c_str();
    }
};

struct NoInitialStateStateMachineException : public StateMachineException
{
    NoInitialStateStateMachineException() : StateMachineException(
            "Invalid State Machine due to no initial State detected") {};
};

struct NoFinalStateStateMachineException : public StateMachineException
{
    NoFinalStateStateMachineException() : StateMachineException(
            "Invalid State Machine due to no final State detected") {};
};

struct MultiInitialStateStateMachineException : public StateMachineException
{
    explicit MultiInitialStateStateMachineException(const std::unordered_set<State*>& invalid_states) :
            StateMachineException(
                    "Invalid State Machine due to no initial State detected\nState detected as initial are : ")
    {
        for (auto& state: invalid_states)
            msg_ += state->toString() + "\n";
    }
};

struct MultiFinalStateStateMachineException : public StateMachineException
{
    explicit MultiFinalStateStateMachineException(const std::unordered_set<State*>& invalid_states) :
            StateMachineException(
                    "Invalid State Machine due to no final State detected\nState detected as final are : ")
    {
        for (auto& state: invalid_states)
            msg_ += state->toString() + "\n";
    }
};

class StateMachine
{
public:
    StateMachine(const std::string& name, int id, uint32_t level = 0);
    StateMachine(const StateMachine& other) = delete;

    EvolveResult_t evolve(Fact* fact);
    EvolveResult_t evolve(StateMachine* state_machine);
//    EvolveResult_t evolve(Action* action);
    EvolveResult_t evolve(Task* task);

    const State* getCurrentState() const { return current_state_; }
    std::string getTypeStr() const { return type_str_; }
    uint32_t getType() const { return type_; }
    std::string getName() const { return full_name_; }
    uint32_t getLevel() const { return level_; }
    TimeStamp_t getLastupdate() const { return last_update_; }
    TimeStamp_t getAge() const { return age_; }
    Variable_t getVar(const std::string& key) const { return variables_.at(key); }
    bool newExplanationAvailable() const { return new_explanations_; }
    float getCompletionRatio() const;
    std::vector<Description_t> getDescription() { return descriptions_; };
    std::vector<std::string> getLiteralVariables();

    bool isFinished() const { return current_state_->isFinalNode(); }
    bool isClosed() const { return closed_; }
    bool isValid() const { return valid_; }





    bool addTransition(const PatternTransitionFact_t& transitionFact);
    bool addTransition(const PatternTransitionStateMachine_t& transitionStateMachine);
    bool addTransition(const HTNTransition_t& transition);

    bool addDescription(const ActionDescription_t& description);

    bool closeStateMachine();

    std::string toString();

    StateMachine* clone(int new_id, int last_state_required = -1);
    void displayVariables();
    std::string describe(bool expl = false);
    std::vector<uint32_t> getIdsFacts() const { return id_facts_involve; }

    bool involveFacts(const std::set<uint32_t>& facts);

    static WordTable types_table;

    static void displayTypesTable();

    std::vector<StateMachine*> getUpdatedStateMachines() { return updated_sub_state_machines_; };

    void addTimeoutTransition(int last_state_required);

    void setId(int new_id);
    std::string getStrStructure();
    void expandProperties(onto::OntologyManipulator* onto_manipulator);
    bool timeEvolution(TimeStamp_t stamp, double time_to_live);
private:

    bool checkIncompletsStateMachines();
    void addState(int id_state);
    void linkStateMachine();
    void insertVariable(const std::string& variable);

    void processInitialState();

    bool
    updateVar(const std::map<std::string, std::string>& remap, const std::map<std::string, Variable_t>& variables_);


    std::string type_str_;
    uint32_t type_;
    uint32_t id_;
    std::string full_name_;
    uint32_t level_;
    bool new_explanations_;

    std::map<std::string, Variable_t> variables_;

    std::vector<Description_t> descriptions_;

    State* current_state_;
    std::map<int, State*> states_;
    int id_initial_state_;
    int id_final_state_;

    std::vector<uint32_t> id_facts_involve;

    bool closed_;
    bool valid_;
    TimeStamp_t age_;
    TimeStamp_t last_update_;
    std::vector<IncompleteStateMachine_t> incompletes_state_machines_;

    std::vector<StateMachine*> updated_sub_state_machines_;


    void linkHTNTransition(int initial_state, int final_state, const HTNTransition_t& transition);
};


} // procedural

#endif //PROCEDURAL_STATE_MACHINE_H
