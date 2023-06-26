#ifndef PROCEDURAL_ACTIONTYPE_H
#define PROCEDURAL_ACTIONTYPE_H

#include <string>
#include <vector>
#include <map>

#include "procedural/core/Types/Fact.h"
#include "procedural/core/Types/Action.h"

namespace procedural {

class ActionType
{
public:
    explicit ActionType(const std::string& name);

    bool addActions(const Action& action);

    bool feed(Fact* fact, TimeStamp_t currentTimestamp);

    std::set<uint32_t> checkCompleteStateMachines(TimeStamp_t currentTimestamp);

    void displayCurrentState();

    void cleanActions(std::set<uint32_t> set_id);
    void clean();

    std::string toString();

    std::string currentState(bool shortVersion = true);

    bool checkSubAction(ActionType* action);

    std::unordered_set<StateMachine*> getCompletesStateMachines() { return complete_state_machines_;};
    std::string getName() const { return name_;};
    bool checkNewExplanation();
    std::vector<StateMachine*> getNewExplanation();

    double maxTtl();

    std::vector<Action> getActions() {return actions_;};
private:
    std::string name_;
    std::unordered_set<StateMachine*> complete_state_machines_;
    std::unordered_set<StateMachine*> updated_sub_state_machines_;
    bool flag_;

    std::vector<Action> actions_;

};

} // namespace procedural

#endif // PROCEDURAL_ACTIONTYPE_H
