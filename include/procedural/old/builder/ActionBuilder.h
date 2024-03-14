#ifndef PROCEDURAL_ACTIONBUILDER_H
#define PROCEDURAL_ACTIONBUILDER_H

#include <vector>
#include <ontologenius/OntologyManipulator.h>
#include "procedural/old/reader/types/ParsedSimpleAction.h"
#include "procedural/old/reader/types/ParsedComposedAction.h"
#include "procedural/old/core/Types/Action.h"
#include "procedural/old/core/ActionRecognition.h"

namespace procedural {

class ActionBuilder
{
public:
    ActionBuilder() : onto_client_(nullptr) {};
    ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                  std::vector<ParsedComposedAction_t>& composed_actions);

    void build(const std::vector<ParsedSimpleAction_t>& simple_actions,
               std::vector<ParsedComposedAction_t>& composed_actions,
               onto::OntologyManipulator* client);

    void display();

    std::vector<Action*> getActions() { return actions_; };
private:
    onto::OntologyManipulator* onto_client_;
    std::vector<ParsedComposedAction_t> incomplete_creation_state_machine_;
    std::vector<Action*> actions_;

    void buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions);
    void buildComposedAction(std::vector<ParsedComposedAction_t>& composed_actions);

    bool completeRemap(SubStateMachine_t& sub_state_machine);
//    void combineActions(const std::vector<ParsedSimpleAction_t>& simple_actions,
//                        const std::vector<ParsedComposedAction_t>& composed_actions);
    void buildIncomplete();
};

} // namespace procedural

#endif // PROCEDURAL_ACTIONBUILDER_H
