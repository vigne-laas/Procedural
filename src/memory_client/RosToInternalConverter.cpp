#include "procedural/memory_client/RosToInternalConverter.h"
#include "procedural/utils/Logger.h"
#include <sstream>
#include <algorithm>

namespace procedural {

// Helper function to infer type from variable name and action arguments
static std::string inferVariableType(const std::string& variable_name,
                                     const std::vector<procedural_interfaces::Argument>& arguments) {
    // Handle special variables
    if (variable_name == "?executor" || variable_name == "?@executor") {
        return "Agent";
    }

    // If it doesn't start with ?, it's a literal - return empty type
    if (variable_name.empty() || variable_name[0] != '?') {
        return "";
    }

    // Extract variable name without ? prefix
    std::string var_name = variable_name.substr(1);

    // Look up in arguments
    for (const auto& arg : arguments) {
        if (arg.literal == var_name) {
            return arg.type;
        }
    }

    // If not found, log warning and return empty
    LOG_DEBUG << "Could not infer type for variable '" << variable_name << "', using empty type";
    return "";
}

ParsedSimpleAction_t RosToInternalConverter::convertToSimpleAction(const procedural_interfaces::Action& action_msg) {
    ParsedSimpleAction_t simple_action;

    simple_action.setType(action_msg.actionName);

    // Convert arguments
    for (const auto& arg : action_msg.arguments) {
        // Use literal (variable name) as key, not value
        // Trim whitespace from literal (parser may add spaces)
        std::string var_name = arg.literal;
        var_name.erase(0, var_name.find_first_not_of(" \t\n\r"));
        var_name.erase(var_name.find_last_not_of(" \t\n\r") + 1);
        simple_action.args.args[var_name] = arg.type;
    }

    // Convert recognition sequences to facts
    int level = 0;
    for (size_t i = 0; i < action_msg.recognition.sequence.size(); ++i) {
        const auto& step = action_msg.recognition.sequence[i];
        ParsedFact_t fact;
        fact.subject = step.subject;
        fact.property = step.predicate;
        fact.object = step.object;
        fact.insertion = !step.is_negative;
        fact.required = step.is_required;

        // Use index as level if step.level is not set (0), otherwise use step.level
        fact.level = (step.level == 0 && i > 0) ? level : step.level;

        // Infer types from arguments
        fact.subject_type = inferVariableType(step.subject, action_msg.arguments);
        fact.object_type = inferVariableType(step.object, action_msg.arguments);

        simple_action.facts.facts_.push_back(fact);

        // Increment level for next iteration
        level = fact.level + 1;
    }

    // Convert descriptions
    for (const auto& triplet : action_msg.description.descriptions) {
        ParsedDescription_t parsed_desc;
        parsed_desc.subject = triplet.subject.literal;
        parsed_desc.property = triplet.property;
        parsed_desc.object = triplet.object.literal;
        simple_action.descriptions.descriptions.push_back(parsed_desc);
    }

    LOG_INFO << "Converted ROS action '" << action_msg.actionName << "' to simple action with "
             << action_msg.recognition.sequence.size() << " recognition facts and "
             << simple_action.descriptions.descriptions.size() << " descriptions";
    return simple_action;
}

ParsedComposedAction_t RosToInternalConverter::convertToComposedAction(const procedural_interfaces::Action& action_msg) {
    ParsedComposedAction_t composed_action;

    composed_action.setType(action_msg.actionName);

    // Convert arguments
    for (const auto& arg : action_msg.arguments) {
        // Use literal (variable name) as key, not type/value
        // Trim whitespace from literal (parser may add spaces)
        std::string var_name = arg.literal;
        var_name.erase(0, var_name.find_first_not_of(" \t\n\r"));
        var_name.erase(var_name.find_last_not_of(" \t\n\r") + 1);
        composed_action.args.args[var_name] = arg.type;
    }

    // Convert recognition sequences to facts
    int level = 0;
    for (size_t i = 0; i < action_msg.recognition.sequence.size(); ++i) {
        const auto& step = action_msg.recognition.sequence[i];
        ParsedFact_t fact;
        fact.subject = step.subject;
        fact.property = step.predicate;
        fact.object = step.object;
        fact.insertion = !step.is_negative;
        fact.required = step.is_required;

        // Use index as level if step.level is not set (0), otherwise use step.level
        fact.level = (step.level == 0 && i > 0) ? level : step.level;

        // Infer types from arguments
        fact.subject_type = inferVariableType(step.subject, action_msg.arguments);
        fact.object_type = inferVariableType(step.object, action_msg.arguments);

        composed_action.pattern.facts.push_back(fact);

        // Increment level for next iteration
        level = fact.level + 1;
    }

    // Set max_level for the pattern
    composed_action.pattern.max_level = level;

    // Convert descriptions
    for (const auto& triplet : action_msg.description.descriptions) {
        ParsedDescription_t parsed_desc;
        parsed_desc.subject = triplet.subject.literal;
        parsed_desc.property = triplet.property;
        parsed_desc.object = triplet.object.literal;
        composed_action.descriptions.descriptions.push_back(parsed_desc);
    }

    LOG_INFO << "Converted ROS action '" << action_msg.actionName << "' to composed action with "
             << action_msg.recognition.sequence.size() << " recognition facts and "
             << composed_action.descriptions.descriptions.size() << " descriptions";
    return composed_action;
}

void RosToInternalConverter::convertActionsToInternal(const std::vector<procedural_interfaces::Action>& action_msgs,
                                                    std::vector<ParsedSimpleAction_t>& simple_actions,
                                                    std::vector<ParsedComposedAction_t>& composed_actions) {
    simple_actions.clear();
    composed_actions.clear();

    int filtered_count = 0;

    for (const auto& action_msg : action_msgs) {
        // Filter out actions without recognition sequences
        if (action_msg.recognition.sequence.empty()) {
            LOG_DEBUG << "Filtering out action '" << action_msg.actionName
                     << "' - no recognition sequence defined (0 facts)";
            filtered_count++;
            continue;
        }

        if (isSimpleAction(action_msg)) {
            simple_actions.push_back(convertToSimpleAction(action_msg));
        } else {
            composed_actions.push_back(convertToComposedAction(action_msg));
        }
    }

    LOG_INFO << "Converted " << simple_actions.size() << " simple actions and "
             << composed_actions.size() << " composed actions";
    if (filtered_count > 0) {
        LOG_INFO << "Filtered out " << filtered_count << " action(s) without recognition sequences";
    }
}

HTNParserd_t RosToInternalConverter::convertTasksToHTN(const std::vector<procedural_interfaces::Task>& task_msgs) {
    HTNParserd_t htn;

    for (const auto& task_msg : task_msgs) {
        if (isPrimitiveTask(task_msg)) {
            htn.actions.push_back(convertToPrimitiveAction(task_msg));
        } else {
            htn.tasks.push_back(convertToAbstractTask(task_msg));
        }
    }

    LOG_INFO << "Converted " << htn.tasks.size() << " abstract tasks and "
             << htn.actions.size() << " primitive actions to HTN";
    return htn;
}

Abstract_task_t RosToInternalConverter::convertToAbstractTask(const procedural_interfaces::Task& task_msg) {
    Abstract_task_t abstract_task;

    abstract_task.name = task_msg.task_name;

    // Convert arguments
    for (const auto& arg : task_msg.arguments) {
        abstract_task.arguments.push_back(convertArgument(arg));
    }

    // Convert methods
    for (const auto& method : task_msg.methods) {
        abstract_task.methods_.push_back(convertMethod(method));
    }

    // Convert effects to goals (since that's how they're used in the structure)
    for (const auto& effect : task_msg.effects) {
        abstract_task.goals.push_back(convertEffect(effect));
    }

    LOG_INFO << "Converted task '" << task_msg.task_name << "' to abstract task with "
             << abstract_task.methods_.size() << " methods";
    return abstract_task;
}

PrimitiveActionParsed_t RosToInternalConverter::convertToPrimitiveAction(const procedural_interfaces::Task& task_msg) {
    PrimitiveActionParsed_t primitive_action;

    primitive_action.name = task_msg.task_name;

    // Convert arguments
    for (const auto& arg : task_msg.arguments) {
        primitive_action.arguments.push_back(convertArgument(arg));
    }

    // Convert effects
    for (const auto& effect : task_msg.effects) {
        primitive_action.effects.simple_effects.push_back(convertEffect(effect));
    }

    LOG_INFO << "Converted task '" << task_msg.task_name << "' to primitive action";
    return primitive_action;
}

bool RosToInternalConverter::isSimpleAction(const procedural_interfaces::Action& action_msg) {
    // Simple heuristic: if there are execution actions, it's likely a simple action
    // Otherwise, consider it composed. This can be refined based on actual action structure.
    return !action_msg.executionActions.empty();
}

bool RosToInternalConverter::isPrimitiveTask(const procedural_interfaces::Task& task_msg) {
    // If it has no methods or is explicitly marked as primitive, it's primitive
    return task_msg.is_primitive || task_msg.methods.empty();
}

Arguments_t RosToInternalConverter::convertArgument(const procedural_interfaces::TaskArgument& arg_msg) {
    Arguments_t argument;
    argument.type = arg_msg.type;
    argument.varname = arg_msg.name;
    argument.name = arg_msg.name;
    return argument;
}

Method_t RosToInternalConverter::convertMethod(const procedural_interfaces::Method& method_msg) {
    Method_t method;

    method.name = method_msg.method_name;

    // Convert preconditions
    for (const auto& precond : method_msg.preconditions) {
        method.preconditions.push_back(convertPrecondition(precond));
    }

    // Convert decomposition to subtasks
    int action_id = 0;
    for (const auto& decomp_str : method_msg.decomposition) {
        parseDecomposition(decomp_str, method.subtask, action_id);
    }

    return method;
}

Expression_t RosToInternalConverter::convertPrecondition(const procedural_interfaces::TaskPrecondition& precond_msg) {
    Expression_t expression;
    expression.subject = precond_msg.subject;
    expression.property = precond_msg.predicate;
    expression.object = precond_msg.object;
    expression.add = !precond_msg.is_negative;
    return expression;
}

Expression_t RosToInternalConverter::convertEffect(const procedural_interfaces::TaskEffect& effect_msg) {
    Expression_t expression;
    expression.subject = effect_msg.subject;
    expression.property = effect_msg.predicate;
    expression.object = effect_msg.object;
    expression.add = effect_msg.is_add;
    return expression;
}

void RosToInternalConverter::parseDecomposition(const std::string& decomposition_str,
                                              Subtask_t& subtask, int& action_id) {
    std::string action_name;
    std::vector<std::string> arguments;

    parseActionCall(decomposition_str, action_name, arguments);

    Ordered_Action_t ordered_action;
    ordered_action.id = action_id++;
    ordered_action.name = action_name;
    ordered_action.arguments = arguments;

    subtask.map_actions[ordered_action.id] = ordered_action;
}

void RosToInternalConverter::parseActionCall(const std::string& decomposition_str,
                                           std::string& action_name,
                                           std::vector<std::string>& arguments) {
    arguments.clear();

    // Find the opening parenthesis
    size_t paren_pos = decomposition_str.find('(');
    if (paren_pos == std::string::npos) {
        action_name = decomposition_str;
        // Remove leading/trailing whitespace
        action_name.erase(0, action_name.find_first_not_of(" \t"));
        action_name.erase(action_name.find_last_not_of(" \t") + 1);
        return;
    }

    action_name = decomposition_str.substr(0, paren_pos);
    // Remove leading/trailing whitespace
    action_name.erase(0, action_name.find_first_not_of(" \t"));
    action_name.erase(action_name.find_last_not_of(" \t") + 1);

    // Find the closing parenthesis
    size_t close_paren = decomposition_str.find(')', paren_pos);
    if (close_paren == std::string::npos) {
        LOG_WARNING << "Malformed action call: " << decomposition_str;
        return;
    }

    // Extract arguments
    std::string args_str = decomposition_str.substr(paren_pos + 1, close_paren - paren_pos - 1);

    if (!args_str.empty()) {
        std::stringstream ss(args_str);
        std::string arg;

        while (std::getline(ss, arg, ',')) {
            // Remove leading/trailing whitespace
            arg.erase(0, arg.find_first_not_of(" \t"));
            arg.erase(arg.find_last_not_of(" \t") + 1);

            if (!arg.empty()) {
                arguments.push_back(arg);
            }
        }
    }
}

// Note: convertRecognitionSequenceToFacts will be implemented later when ROS messages support recognition

} // namespace procedural