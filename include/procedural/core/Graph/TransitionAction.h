#ifndef PROCEDURAL_TRANSITIONACTION_H
#define PROCEDURAL_TRANSITIONACTION_H

#include <ontologenius/clients/ontologyClients/IndividualClient.h>
#include "HTNTransition.h"
#include "procedural/core/Types/Variable.h"


namespace procedural {
class Action;
class TransitionAction
{
public:
    explicit TransitionAction(uint32_t id_action, int next_state, const std::map<std::string, std::string>& map);

    TransitionAction(const TransitionAction& transition, int id_next_state);
    void linkVariables(std::map<std::string,Variable_t>& variables);
    bool match(Action* action);
    std::string toString() const;

    void setOntologyClient(onto::IndividualClient* indiv_client);
private:
    uint32_t action_id_;
    std::map<std::string,std::string> arguments_;
    std::map<std::string,Variable_t*> variables_;
    int id_next_state_;
};

} // procedural

#endif //PROCEDURAL_TRANSITIONACTION_H
