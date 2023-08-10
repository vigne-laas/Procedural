#ifndef PROCEDURAL_HTNSTATEMACHINE_H
#define PROCEDURAL_HTNSTATEMACHINE_H
#include <string>
#include "procedural/core/Types/Action.h"
#include "procedural/core/Graph/HTNTransition.h"
#include "procedural/core/Types/Task.h"
#include "HtnState.h"


namespace procedural {
struct HTNStateMachineException : public std::exception
{
    std::string msg_;
    explicit HTNStateMachineException(const std::string& msg) : msg_(msg)
    {}
    const char* what() const throw()
    {
        return msg_.c_str();
    }
};
struct NoInitialHTNStateStateMachineException : public HTNStateMachineException
{
    NoInitialHTNStateStateMachineException() : HTNStateMachineException("Invalid State Machine due to no initial State detected"){};
};

struct MultiInitialStateHTNStateMachineException : public HTNStateMachineException
{
    explicit MultiInitialStateHTNStateMachineException(const std::unordered_set<HTNState*>& invalid_states) :
            HTNStateMachineException("Invalid State Machine due to no initial State detected\nState detected as initial are : ")
    {
        for (auto& state: invalid_states)
            msg_ += state->toString() + "\n";
    }
};

class HTNStateMachine
{
public:
    HTNStateMachine(const std::string name,int id);
    HTNStateMachine(const HTNStateMachine& other)=delete;

    bool evolve(Action* action);
    bool evolve(Task* task);

    void addTransition(const procedural::HTNTransition_t& transition);
    bool close();
private:
    std::string name_;
    std::string full_name_;
    int id_;
    uint32_t type_;

    std::map<std::string, Variable_t> variables_;

    HTNState* current_state_{};
    std::map<int, HTNState*> states_;
    int id_initial_state_{};

    bool closed_{};
    bool valid_{};
    TimeStamp_t age_;
    TimeStamp_t last_update_;


    static WordTable types_table;


    void addState(int state);
    void linkStateMachine();
    void processInitialState();
};

} // procedural

#endif //PROCEDURAL_HTNSTATEMACHINE_H
