#ifndef PROCEDURAL_ACTION_ROS_CONVERTER_H
#define PROCEDURAL_ACTION_ROS_CONVERTER_H

#include <procedural_interfaces/Action.h>
#include <procedural_interfaces/action_t.h>
#include <procedural/task_recognition/Reader/domainTypes/ParsedHTN.h>  // Must be before CommitmentConverter
#include <procedural/memory/CommitmentConverter.h>

namespace procedural {

/**
 * @brief Converts Action_t to ROS Action message with full commitment support
 *
 * This function wraps the standard toRosMsg() method and adds commitment conversion
 * where the CommitmentBlock_t type is fully defined.
 */
inline procedural_interfaces::Action convertActionToRos(const procedural_interfaces::Action_t& action)
{
    // Use the standard toRosMsg() conversion
    procedural_interfaces::Action msg = action.toRosMsg();

    // Add commitment conversion if commitments are present
    if (action.has_commitments && action.commitments) {
        msg.commitment_info = CommitmentConverter::convertToRosMessage(*action.commitments, action.name);
    }

    return msg;
}

} // namespace procedural

#endif // PROCEDURAL_ACTION_ROS_CONVERTER_H
