#ifndef PROCEDURAL_COMMITMENT_CONVERTER_H
#define PROCEDURAL_COMMITMENT_CONVERTER_H

#include <procedural_interfaces/CommitmentInfo.h>
#include <procedural_interfaces/CommitmentCondition.h>
#include <procedural/task_recognition/Reader/domainTypes/ParsedHTN.h>

namespace procedural {

/**
 * @brief Converts internal commitment structures to ROS messages
 *
 * This class provides static methods to convert commitment blocks and conditions
 * from the internal parser representation to ROS message format for IPC.
 */
class CommitmentConverter {
public:
    /**
     * @brief Convert CommitmentBlock_t to ROS CommitmentInfo message
     * @param commitment_block Internal commitment block structure
     * @param action_name Name of the action this commitment belongs to
     * @return CommitmentInfo ROS message
     */
    static procedural_interfaces::CommitmentInfo convertToRosMessage(
        const ::procedural::CommitmentBlock_t& commitment_block,
        const std::string& action_name)
    {
        procedural_interfaces::CommitmentInfo msg;

        // Basic info
        msg.action_name = action_name;
        msg.commitment_id = action_name + "_commitment";  // Simple ID for now
        msg.status = "PENDING";  // Initial status

        // Convert instrumental conditions
        for (const auto& cond : commitment_block.instrumental) {
            msg.instrumental_conditions.push_back(convertCondition(cond, "INSTRUMENTAL"));
        }

        // Convert engagement conditions
        for (const auto& cond : commitment_block.engagement) {
            msg.engagement_conditions.push_back(convertCondition(cond, "ENGAGEMENT"));
        }

        // Convert common ground conditions
        for (const auto& cond : commitment_block.common_ground) {
            msg.common_ground_conditions.push_back(convertCondition(cond, "COMMON_GROUND"));
        }

        // Set reactions
        msg.on_instrumental_failure = commitment_block.on_instrumental_failure;
        msg.on_engagement_failure = commitment_block.on_engagement_failure;
        msg.on_common_ground_failure = commitment_block.on_common_ground_failure;

        // Set recovery strategy
        msg.recovery_mode = commitment_block.recovery_strategy.mode;
        msg.max_attempts = commitment_block.recovery_strategy.max_attempts;
        msg.timeout = commitment_block.recovery_strategy.timeout;
        msg.current_attempts = 0;  // Initialize to 0

        // Initialize times (will be set by commitment monitor)
        msg.commitment_time = ros::Time(0);
        msg.last_check_time = ros::Time(0);
        msg.duration = 0.0;

        return msg;
    }

private:
    /**
     * @brief Convert CommitmentCondition_t to ROS CommitmentCondition message
     * @param condition Internal condition structure
     * @param type Condition type ("INSTRUMENTAL", "ENGAGEMENT", "COMMON_GROUND")
     * @return CommitmentCondition ROS message
     */
    static procedural_interfaces::CommitmentCondition convertCondition(
        const ::procedural::CommitmentCondition_t& condition,
        const std::string& type)
    {
        procedural_interfaces::CommitmentCondition msg;

        msg.condition_type = type;
        msg.sparql_query = condition.sparql_query;
        msg.description = condition.description;
        msg.is_satisfied = false;  // Initial state, will be updated by monitor
        msg.for_clause = condition.for_clause;  // Preserve FOR clause from parser

        return msg;
    }
};

} // namespace procedural

#endif // PROCEDURAL_COMMITMENT_CONVERTER_H
