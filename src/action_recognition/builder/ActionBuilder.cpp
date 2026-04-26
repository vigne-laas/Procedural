#include "procedural/action_recognition/builder/ActionBuilder.h"
#include "procedural/utils/Logger.h"

namespace procedural {


ActionBuilder::ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                             const std::vector<ParsedComposedAction_t>& composed_actions, const std::string& path)
        : onto_client_(nullptr), actions_(),
          incomplete_creation_state_machine_()
{
    if (!build(simple_actions, composed_actions, path)) {
        LOG_ERROR << "Failed to build actions";
        throw ActionBuilderException("Failed to build actions");
    }
}

ActionBuilder::ActionBuilder(const std::vector<ParsedSimpleAction_t>& simple_actions,
                             const std::vector<ParsedComposedAction_t>& composed_actions,
                             onto::OntologyManipulator* client, const std::string& path) : onto_client_(client),
                                                                                           actions_(),
                                                                                           incomplete_creation_state_machine_()
{
    if (!build(simple_actions, composed_actions, path)) {
        LOG_ERROR << "Failed to build actions";
        throw ActionBuilderException("Failed to build actions");
    }
}

bool ActionBuilder::build(const std::vector<ParsedSimpleAction_t>& simple_actions,
                          const std::vector<ParsedComposedAction_t>& composed_actions, const std::string& path)
{
    LOG_INFO << "ActionBuilder::build starting with " << simple_actions.size()
             << " simple actions and " << composed_actions.size() << " composed actions";

    if (not checkAction(simple_actions, composed_actions)) {
        LOG_ERROR << "Action validation failed";
        return false;
    }

    LOG_INFO << "Building simple actions...";
    buildSimpleAction(simple_actions, path);

    LOG_INFO << "Building composed actions...";
    if (buildComposedAction(composed_actions, path)) {
        LOG_INFO << "ActionBuilder::build completed successfully. Built " << actions_.size() << " actions total";
        return true;
    }
    LOG_ERROR << "Failed to build composed actions";
    throw ActionBuilderException("Failed to build composed actions");
}

void ActionBuilder::buildSimpleAction(const std::vector<ParsedSimpleAction_t>& simple_actions, const std::string& path)
{
    LOG_INFO << "Building " << simple_actions.size() << " simple actions";
    for (const auto& simple_action: simple_actions) {
        LOG_INFO << "  Building simple action: " << simple_action.getName() << " (type: " << simple_action.type << ")";
        auto action = new Action(simple_action.type);
        if (action->build(simple_action, path)) {
            actions_.push_back(action);
            action_build.push_back(action->getName());
            LOG_INFO << "  ✓ Successfully built action: " << action->getName();
        } else {
            LOG_ERROR << "  ✗ Failed to build simple action: " << simple_action.type;
            throw ActionBuilderException("Failed to build simple action : " + simple_action.type);
        }

    }
}

bool ActionBuilder::buildComposedAction(const std::vector<ParsedComposedAction_t>& composed_actions, const std::string& path)
{
    auto toBuildActions = composed_actions;
    std::vector<ParsedComposedAction_t> incomplete_action_;
    int nb_built_actions_;
    do {
        nb_built_actions_ = 0;
        for (auto& composed_action: toBuildActions) {
            if (checkAlreadyBuiltAction(composed_action.pattern.sub_state_machines)) {
                auto action = new Action(composed_action.getName());
                if (action->build(composed_action, actions_, path)) {
                    action->getFactory()->saveDot(path);
                    actions_.push_back(action);
                    action_build.push_back(action->getName());
                } else {
                    throw ActionBuilderException("Failed to build composed action : " + composed_action.getName());
                }
                nb_built_actions_++;
            } else {
                incomplete_action_.push_back(composed_action);
            }
        }
        toBuildActions = incomplete_action_;
        incomplete_action_.clear();
    } while (nb_built_actions_ != 0);
    return toBuildActions.empty();
}

bool ActionBuilder::completeRemap(SubStateMachine_t& sub_state_machine)
{
//    for (auto& action: actions_) {
//        if (action->getType() == sub_state_machine.getType()) {
//            action->remap(sub_state_machine);
//            return true;
//        }
//    }
    return false;
}

void ActionBuilder::buildIncomplete()
{
//    for (auto& sub_state_machine: incomplete_creation_state_machine_) {
//        if (!completeRemap(sub_state_machine)) {
//            std::cerr << "Could not find action: " << sub_state_machine.getType() << std::endl;
//        }
//    }
}

void ActionBuilder::display() const
{
//    for (const auto& action: actions_) {
//        action->display();
//    }
}

bool ActionBuilder::checkAction(const std::vector<ParsedSimpleAction_t>& simple_actions,
                                const std::vector<ParsedComposedAction_t>& composed_actions)
{
    std::set<std::string> action_types;

    // Check simple actions for empty recognition facts
    for (const auto& simple_action: simple_actions) {
        action_types.insert(simple_action.type);

        if (simple_action.facts.facts_.empty()) {
            LOG_WARNING << "Simple action '" << simple_action.getName() << "' has no recognition facts";
            LOG_WARNING << "This action should have been filtered during conversion";
        }
    }

    for (const auto& composed_action: composed_actions) {
        action_types.insert(composed_action.getName());
    }

    std::set<std::string> action_needed_by_composed_action_types;
    for (const auto& composed_action: composed_actions) {
        // Check if composed action has neither facts nor sub-state machines
        if (composed_action.pattern.facts.empty() && composed_action.pattern.sub_state_machines.empty()) {
            LOG_WARNING << "Composed action '" << composed_action.getName() << "' has no facts and no sub-state machines";
            LOG_WARNING << "This action can be represented by a task instead of an action";
        } else if (composed_action.pattern.facts.empty()) {
            LOG_DEBUG << "Composed action '" << composed_action.getName() << "' has no facts (relies on sub-state machines)";
        }

        for (const auto& sub_machine: composed_action.pattern.sub_state_machines) {
            action_needed_by_composed_action_types.insert(sub_machine.type);
        }
    }

    for (const auto& action: action_needed_by_composed_action_types) {
        if (action_types.find(action) == action_types.end()) {
            LOG_ERROR << "Action '" << action << "' needed by composed action not found";
            return false;
        }
    }
    return true;
}

bool ActionBuilder::checkAlreadyBuiltAction(std::vector<SubStateMachine_t> subMachines)
{
    auto res = std::all_of(subMachines.begin(), subMachines.end(), [this](const SubStateMachine_t& subMachine) {
        return std::find(action_build.begin(), action_build.end(), subMachine.type) != action_build.end();
    });
    return res;
}


} // action_recognition