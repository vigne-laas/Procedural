#ifndef PROCEDURAL_ACTIONTYPE_H
#define PROCEDURAL_ACTIONTYPE_H

#include <string>
#include <vector>
#include <map>

#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/Action.h"

namespace procedural {

class ActionMethod
{
public:
    explicit ActionMethod(const std::string& name);

    bool addActions(Action* action);

    bool feed(Fact* fact, TimeStamp_t currentTimestamp);

    std::set<uint32_t> checkCompleteStateMachines(TimeStamp_t currentTimestamp);

    void displayCurrentState();

    void cleanActions(std::set<uint32_t> set_id);
    void clean();

    std::string toString();

    std::string currentState(bool shortVersion = true);

    bool checkSubAction(ActionMethod* action);

    std::unordered_set<StateMachine*> getCompletesStateMachines() { return complete_state_machines_;};
    std::string getName() const { return name_;};
    bool checkNewExplanation();
    std::vector<StateMachine*> getNewExplanation();

    double maxTtl();

    std::vector<Action*> getActions() {return actions_;};

    uint32_t getId() { return id_; };
    static WordTable action_method_types;
private:
    std::string name_;
    std::unordered_set<StateMachine*> complete_state_machines_;
    std::unordered_set<StateMachine*> updated_sub_state_machines_;
    bool flag_;

    std::vector<Action*> actions_;

    uint32_t id_;
};

} // namespace procedural

#endif // PROCEDURAL_ACTIONTYPE_H
