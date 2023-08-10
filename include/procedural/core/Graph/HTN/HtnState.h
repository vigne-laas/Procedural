#ifndef PROCEDURAL_HTNSTATE_H
#define PROCEDURAL_HTNSTATE_H

#include <string>
#include "procedural/core/Types/Task.h"
#include "procedural/core/Graph/TransitionAction.h"
#include "procedural/core/Graph/TransitionTask.h"

namespace procedural {

class HTNState
{
public:
    explicit HTNState(const std::string& name, int id);

    HTNState* evolve(Action* action);
    HTNState* evolve(Task* task);

    void addTransition(const TransitionAction& transition, HTNState* next_states);
    void addTransition(const TransitionTask& transition, HTNState* next_states);

    void linkVariables(std::map<std::string, Variable_t>& variables_);
    bool isFinalNode() const { return nexts_actions_.empty() && nexts_task_.empty(); }
    void setInitialNode() { initial_node_ = true; };
    int getId() const { return id_; };
    std::string toString() const;

    const std::vector<std::pair<TransitionAction, HTNState*>> getNextsActions() const { return nexts_actions_; };
    const std::vector<std::pair<TransitionTask, HTNState*>> getNextsTasks() const { return nexts_task_; };

private:

    int id_;
    std::string name_;
    bool initial_node_;
    std::vector<std::pair<TransitionAction, HTNState*>> nexts_actions_;
    std::vector<std::pair<TransitionTask, HTNState*>> nexts_task_;

};

} // procedural

#endif //PROCEDURAL_HTNSTATE_H
