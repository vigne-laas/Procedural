#ifndef PROCEDURAL_SPECIALIZEDACTION_H
#define PROCEDURAL_SPECIALIZEDACTION_H

#include <vector>
#include "procedural/core/Types/PatternTransitionFact.h"
#include "procedural/core/Graph/StateMachine.h"
#include "procedural/core/Types/ActionDescription.h"
#include "procedural/core/Types/Fact.h"


namespace procedural {
class ActionType;
class Action
{
public:
    Action(const std::string& name,
           const std::vector<PatternTransitionFact_t>& patterns,
           const std::vector<PatternTransitionNetwork_t>& patterns_network,
           const std::vector<ActionDescription_t>& descriptions,
           int last_state_required,
           ObjectPropertyClient* object_client,
           double ttl=30);

    bool isValid() const { return is_valid_; }
    double getTtl() const {return time_to_live_;}
    std::string getName() const { return name_;}
    int getNextId();
    std::set<uint32_t> checkNetwork(TimeStamp_t current_timestamp);

    void cleanInvolve(const std::set<uint32_t>& list_valid_facts);
    void clean();

    std::unordered_set<StateMachine*> getCompleteNetwork() { return complete_networks_;};

    bool feed(Fact* fact);

    std::string toString();

    std::string currentState(bool shortVersion = true);

    bool checksubAction(ActionType* action);
    bool checkNewUpdatedSubNetwork(){return updated_networks.empty() == false;};
    std::vector<StateMachine*> getUpdatedNetworks() {return updated_networks;};

    std::vector<std::string> getLiteralVariables() {return state_machine_factory_->getLiteralVariables();};


    std::string getStrStructure();
private:
    int id_;

    std::string name_;
    StateMachine* state_machine_factory_; //TODO issue when i try without *

    std::unordered_set<StateMachine*> networks_;
    std::unordered_set<StateMachine*> complete_networks_;
    std::vector<StateMachine*> updated_networks;


    bool is_valid_;
    bool evolve_sub_action;
    double time_to_live_;

    int last_state_required_;

};

} // namespace procedural

#endif //PROCEDURAL_SPECIALIZEDACTION_H
