#ifndef PROCEDURAL_ROS_TO_INTERNAL_CONVERTER_H
#define PROCEDURAL_ROS_TO_INTERNAL_CONVERTER_H

#include <procedural_interfaces/Action.h>
#include <procedural_interfaces/Task.h>
#include <procedural_interfaces/Method.h>
#include <procedural_interfaces/TaskArgument.h>
#include <procedural_interfaces/TaskPrecondition.h>
#include <procedural_interfaces/TaskEffect.h>
#include <procedural_interfaces/Recognition.h>
#include <procedural_interfaces/RecognitionSequenceStep.h>
#include <procedural_interfaces/RecognitionParameters.h>
#include <procedural_interfaces/action_t.h>

#include "procedural/action_recognition/reader/types/ParsedSimpleAction.h"
#include "procedural/action_recognition/reader/types/ParsedComposedAction.h"
#include "procedural/task_recognition/Reader/domainTypes/ParsedHTN.h"

namespace procedural {

/**
 * @brief Converts ROS message types to internal parser structures
 *
 * This class provides static methods to convert ROS Action and Task messages
 * to the internal structures used by the action and task recognition modules.
 */
class RosToInternalConverter {
public:
    /**
     * @brief Convert ROS Action message to ParsedSimpleAction_t
     * @param action_msg ROS Action message
     * @return ParsedSimpleAction_t structure
     */
    static ParsedSimpleAction_t convertToSimpleAction(const procedural_interfaces::Action& action_msg);

    /**
     * @brief Convert ROS Action message to ParsedComposedAction_t
     * @param action_msg ROS Action message
     * @return ParsedComposedAction_t structure
     */
    static ParsedComposedAction_t convertToComposedAction(const procedural_interfaces::Action& action_msg);

    /**
     * @brief Convert multiple ROS Action messages to internal action structures
     * @param action_msgs Vector of ROS Action messages
     * @param simple_actions Output vector for simple actions
     * @param composed_actions Output vector for composed actions
     */
    static void convertActionsToInternal(const std::vector<procedural_interfaces::Action>& action_msgs,
                                       std::vector<ParsedSimpleAction_t>& simple_actions,
                                       std::vector<ParsedComposedAction_t>& composed_actions);

    /**
     * @brief Convert ROS Task messages to HTNParserd_t structure
     * @param task_msgs Vector of ROS Task messages
     * @return HTNParserd_t structure containing all tasks and actions
     */
    static HTNParserd_t convertTasksToHTN(const std::vector<procedural_interfaces::Task>& task_msgs);

    /**
     * @brief Convert ROS Task message to Abstract_task_t
     * @param task_msg ROS Task message
     * @return Abstract_task_t structure
     */
    static Abstract_task_t convertToAbstractTask(const procedural_interfaces::Task& task_msg);

    /**
     * @brief Convert ROS Task message to PrimitiveActionParsed_t (if it's a primitive task)
     * @param task_msg ROS Task message
     * @return PrimitiveActionParsed_t structure
     */
    static PrimitiveActionParsed_t convertToPrimitiveAction(const procedural_interfaces::Task& task_msg);

    /**
     * @brief Check if action message represents a simple action
     * @param action_msg ROS Action message
     * @return true if it's a simple action, false if composed
     */
    static bool isSimpleAction(const procedural_interfaces::Action& action_msg);

    /**
     * @brief Check if task message represents a primitive task/action
     * @param task_msg ROS Task message
     * @return true if it's primitive, false if abstract task
     */
    static bool isPrimitiveTask(const procedural_interfaces::Task& task_msg);

private:
    /**
     * @brief Convert ROS Argument to Arguments_t
     * @param arg_msg ROS Argument message
     * @return Arguments_t structure
     */
    static Arguments_t convertArgument(const procedural_interfaces::TaskArgument& arg_msg);

    /**
     * @brief Convert ROS Method to Method_t
     * @param method_msg ROS Method message
     * @return Method_t structure
     */
    static Method_t convertMethod(const procedural_interfaces::Method& method_msg);

    /**
     * @brief Convert ROS TaskPrecondition to Expression_t
     * @param precond_msg ROS TaskPrecondition message
     * @return Expression_t structure
     */
    static Expression_t convertPrecondition(const procedural_interfaces::TaskPrecondition& precond_msg);

    /**
     * @brief Convert ROS TaskEffect to Expression_t
     * @param effect_msg ROS TaskEffect message
     * @return Expression_t structure
     */
    static Expression_t convertEffect(const procedural_interfaces::TaskEffect& effect_msg);

    /**
     * @brief Parse decomposition string to ordered actions
     * @param decomposition_str Decomposition string from ROS message
     * @param subtask Output subtask structure
     * @param action_id Current action ID counter
     */
    static void parseDecomposition(const std::string& decomposition_str,
                                 Subtask_t& subtask, int& action_id);

    /**
     * @brief Extract action name and arguments from decomposition string
     * @param decomposition_str Decomposition string (e.g., "action_name(arg1, arg2)")
     * @param action_name Output action name
     * @param arguments Output arguments vector
     */
    static void parseActionCall(const std::string& decomposition_str,
                              std::string& action_name,
                              std::vector<std::string>& arguments);

    // Note: convertRecognitionSequenceToFacts will be declared later when ROS messages support recognition
};

} // namespace procedural

#endif // PROCEDURAL_ROS_TO_INTERNAL_CONVERTER_H