#ifndef PROCEDURAL_ACTIONBUILDER_H
#define PROCEDURAL_ACTIONBUILDER_H

#include <vector>
#include <ontologenius/OntologyManipulator.h>
#include "procedural/action_recognition/reader/types/ParsedSimpleAction.h"
#include "procedural/action_recognition/reader/types/ParsedComposedAction.h"
#include "procedural/action_recognition/core/internal_structures/Action.h"

namespace procedural {

struct ActionBuilderException : public std::exception {
    std::string message_;

    explicit ActionBuilderException(const std::string& message) : message_(message) {};

    const char* what() const noexcept override { return message_.c_str(); };
};

class ActionBuilder {
public:
    ActionBuilder() = default;


    ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                  const std::vector<ParsedComposedAction_t>& composed_actions,const std::string& path = "");

    ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                  const std::vector<ParsedComposedAction_t>& composed_actions,
                  onto::OntologyManipulator* client, const std::string& path = "");

    bool build(const std::vector<ParsedSimpleAction_t>& simple_actions,
               const std::vector<ParsedComposedAction_t>& composed_actions, const std::string& path = "");

    void display() const;

    std::vector<Action*> getActions() const { return actions_; };
private:
    std::shared_ptr<onto::OntologyManipulator> onto_client_;
    std::vector<ParsedComposedAction_t> incomplete_creation_state_machine_;
    std::vector<Action*> actions_;
    std::vector<std::string> action_build;

    void buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions, const std::string& path = "");

    bool buildComposedAction(const std::vector<ParsedComposedAction_t>& composed_actions, const std::string& path = "");

    bool completeRemap(SubStateMachine_t& sub_state_machine);

    void buildIncomplete();

    bool checkAction(const std::vector<ParsedSimpleAction_t>& simple_actions,
                     const std::vector<ParsedComposedAction_t>& composed_actions);

    bool checkAlreadyBuiltAction(std::vector<SubStateMachine_t> subMachines);
};


} // action_recognition

#endif //PROCEDURAL_ACTIONBUILDER_H
