/**
 * @file commitment_monitor.cpp
 * @brief Real-time SPARQL-based commitment monitoring node
 *
 * This node monitors commitment conditions during action execution by:
 * - Subscribing to commitment monitoring requests from MissionManager
 * - Registering SPARQL conditions with Yggdrasil for continuous evaluation
 * - Publishing CommitmentEvent messages when violations are detected
 * - Cleaning up registrations when actions complete
 */

#include <ros/ros.h>
#include <procedural_interfaces/CommitmentEvent.h>
#include <procedural_interfaces/CommitmentInfo.h>
#include <procedural_interfaces/CommitmentMonitoringRequest.h>
#include <yggdrasil_interfaces/RegisterEventService.h>
#include <yggdrasil_interfaces/UnRegisterEventService.h>
#include <yggdrasil_interfaces/Event.h>
#include <procedural/ResponsibilityAnalyzer.h>
#include <map>
#include <vector>
#include <string>

class CommitmentMonitor {
public:
    CommitmentMonitor(ros::NodeHandle& nh)
        : nh_(nh), commitment_counter_(0) {

        // Subscribe to monitoring requests from MissionManager
        monitoring_request_sub_ = nh_.subscribe("/commitment/monitoring_request", 10,
            &CommitmentMonitor::monitoringRequestCallback, this);

        // Subscribe to Yggdrasil events for condition violations
        yggdrasil_events_sub_ = nh_.subscribe("/yggdrasil/events", 10,
            &CommitmentMonitor::yggdrasilEventCallback, this);

        // Subscribe to commitment lifecycle events to track active commitments
        commitment_events_sub_ = nh_.subscribe("/commitment/events", 10,
            &CommitmentMonitor::commitmentEventCallback, this);

        // Publisher for violation events
        commitment_events_pub_ = nh_.advertise<procedural_interfaces::CommitmentEvent>(
            "/commitment/events", 10);

        // Service clients for Yggdrasil
        register_event_client_ = nh_.serviceClient<yggdrasil_interfaces::RegisterEventService>(
            "/yggdrasil/register_event");
        unregister_event_client_ = nh_.serviceClient<yggdrasil_interfaces::UnRegisterEventService>(
            "/yggdrasil/unregister_event");

        ROS_INFO("✅ CommitmentMonitor initialized");
        ROS_INFO("   - Listening for monitoring requests on /commitment/monitoring_request");
        ROS_INFO("   - Monitoring commitments via SPARQL conditions");
        ROS_INFO("   - Publishing violations to /commitment/events");
    }

    void monitorCommitment(const std::string& commitment_id,
                          const std::string& action_name,
                          const std::string& agent_id,
                          const procedural_interfaces::CommitmentInfo& commitment_info) {

        ROS_INFO("=== STARTING COMMITMENT MONITORING ===");
        ROS_INFO("Commitment ID: %s", commitment_id.c_str());
        ROS_INFO("Action: %s", action_name.c_str());
        ROS_INFO("Agent: %s", agent_id.c_str());

        // Store commitment metadata
        CommitmentMetadata metadata;
        metadata.commitment_id = commitment_id;
        metadata.action_name = action_name;
        metadata.agent_id = agent_id;
        metadata.commitment_info = commitment_info;

        // Register INSTRUMENTAL conditions
        for (const auto& condition : commitment_info.instrumental_conditions) {
            int64_t reg_id = registerCondition(commitment_id, "INSTRUMENTAL",
                condition, commitment_info.on_instrumental_failure);
            if (reg_id > 0) {
                metadata.registration_ids.push_back(reg_id);
                condition_to_commitment_[reg_id] = {commitment_id, "INSTRUMENTAL", condition.for_clause};
            }
        }

        // Register ENGAGEMENT conditions
        for (const auto& condition : commitment_info.engagement_conditions) {
            int64_t reg_id = registerCondition(commitment_id, "ENGAGEMENT",
                condition, commitment_info.on_engagement_failure);
            if (reg_id > 0) {
                metadata.registration_ids.push_back(reg_id);
                condition_to_commitment_[reg_id] = {commitment_id, "ENGAGEMENT", condition.for_clause};
            }
        }

        // Register COMMON_GROUND conditions
        for (const auto& condition : commitment_info.common_ground_conditions) {
            int64_t reg_id = registerCondition(commitment_id, "COMMON_GROUND",
                condition, commitment_info.on_common_ground_failure);
            if (reg_id > 0) {
                metadata.registration_ids.push_back(reg_id);
                condition_to_commitment_[reg_id] = {commitment_id, "COMMON_GROUND", condition.for_clause};
            }
        }

        // Store metadata
        active_commitments_[commitment_id] = metadata;

        ROS_INFO("✓ Registered %zu SPARQL conditions for monitoring",
                 metadata.registration_ids.size());
        ROS_INFO("======================================");
    }

private:
    struct CommitmentMetadata {
        std::string commitment_id;
        std::string action_name;
        std::string agent_id;
        procedural_interfaces::CommitmentInfo commitment_info;
        std::vector<int64_t> registration_ids;
    };

    struct ConditionMapping {
        std::string commitment_id;
        std::string condition_type;
        std::string for_clause;  // For responsibility attribution
    };

    ros::NodeHandle& nh_;
    ros::Subscriber monitoring_request_sub_;
    ros::Subscriber yggdrasil_events_sub_;
    ros::Subscriber commitment_events_sub_;
    ros::Publisher commitment_events_pub_;
    ros::ServiceClient register_event_client_;
    ros::ServiceClient unregister_event_client_;

    std::map<std::string, CommitmentMetadata> active_commitments_;  // commitment_id -> metadata
    std::map<int64_t, ConditionMapping> condition_to_commitment_;   // registration_id -> commitment info
    int commitment_counter_;
    procedural::ResponsibilityAnalyzer responsibility_analyzer_;  // KISS attribution analyzer

    void monitoringRequestCallback(const procedural_interfaces::CommitmentMonitoringRequest::ConstPtr& msg) {
        ROS_INFO("📨 Received monitoring request for commitment: %s", msg->commitment_id.c_str());

        monitorCommitment(msg->commitment_id, msg->action_name, msg->agent_id, msg->commitment_info);
    }

    int64_t registerCondition(const std::string& commitment_id,
                              const std::string& condition_type,
                              const procedural_interfaces::CommitmentCondition& condition,
                              const std::string& recovery_action) {

        yggdrasil_interfaces::RegisterEventService srv;
        srv.request.event_name = commitment_id + "_" + condition_type + "_" + std::to_string(++commitment_counter_);
        srv.request.source = "commitment_monitor";
        srv.request.condition = condition.sparql_query;
        srv.request.call_on_deactivate = true;  // We want to know when condition becomes FALSE

        if (register_event_client_.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("  [OK] Registered %s condition (ID: %ld)",
                         condition_type.c_str(), srv.response.registration_id);
                ROS_INFO("       Query: %s", condition.sparql_query.c_str());
                return srv.response.registration_id;
            } else {
                ROS_WARN("  [FAIL] Failed to register %s condition", condition_type.c_str());
            }
        } else {
            ROS_ERROR("  [ERROR] Service call failed for %s condition", condition_type.c_str());
        }

        return -1;
    }

    void yggdrasilEventCallback(const yggdrasil_interfaces::Event::ConstPtr& msg) {
        // Check if this is a commitment condition violation (deactivation)
        if (!msg->is_deactivation) {
            return;  // We only care about condition becoming false
        }

        auto it = condition_to_commitment_.find(msg->registration_id);
        if (it == condition_to_commitment_.end()) {
            return;  // Not a commitment condition
        }

        std::string commitment_id = it->second.commitment_id;
        std::string condition_type = it->second.condition_type;

        auto commit_it = active_commitments_.find(commitment_id);
        if (commit_it == active_commitments_.end()) {
            return;  // Commitment no longer active
        }

        CommitmentMetadata& metadata = commit_it->second;

        ROS_WARN("========================================");
        ROS_WARN("⚠ COMMITMENT CONDITION VIOLATED ⚠");
        ROS_WARN("Commitment ID: %s", commitment_id.c_str());
        ROS_WARN("Action: %s", metadata.action_name.c_str());
        ROS_WARN("Condition Type: %s", condition_type.c_str());

        // Determine recovery action
        std::string recovery_action;
        if (condition_type == "INSTRUMENTAL") {
            recovery_action = metadata.commitment_info.on_instrumental_failure;
        } else if (condition_type == "ENGAGEMENT") {
            recovery_action = metadata.commitment_info.on_engagement_failure;
        } else if (condition_type == "COMMON_GROUND") {
            recovery_action = metadata.commitment_info.on_common_ground_failure;
        }

        // Analyze responsibility attribution using KISS approach
        std::string for_clause = it->second.for_clause;
        procedural::AttributionResult attribution =
            responsibility_analyzer_.analyzeForClause(for_clause);

        // Publish violation event with attribution
        procedural_interfaces::CommitmentEvent event;
        event.event_type = "CONDITION_VIOLATED";
        event.commitment_id = commitment_id;
        event.agent_id = metadata.agent_id;
        event.action_name = metadata.action_name;
        event.condition_type = condition_type;
        event.reaction_action = recovery_action;
        event.timestamp = ros::Time::now();

        // Add responsibility attribution fields
        event.responsibility_attribution = attribution.attribution;
        event.attribution_confidence = attribution.confidence;
        event.failure_details = attribution.failure_details;

        if (!recovery_action.empty()) {
            ROS_WARN("→ Recovery Action: %s", recovery_action.c_str());
        }
        ROS_WARN("→ Responsibility: %s (confidence: %.2f)",
                 attribution.attribution.c_str(), attribution.confidence);
        ROS_WARN("→ Details: %s", attribution.failure_details.c_str());

        commitment_events_pub_.publish(event);

        ROS_WARN("✓ Published VIOLATED event with attribution");
        ROS_WARN("========================================");
    }

    void commitmentEventCallback(const procedural_interfaces::CommitmentEvent::ConstPtr& msg) {
        // Listen for MADE events to start monitoring
        if (msg->event_type == "MADE") {
            // Extract commitment info from metadata (if provided in the future)
            // For now, this is a placeholder for when MissionManager publishes full info
            ROS_INFO("📝 Commitment MADE event received: %s", msg->commitment_id.c_str());
        }

        // Clean up when commitment is fulfilled, violated, or dropped
        if (msg->event_type == "FULFILLED" || msg->event_type == "DROPPED") {
            cleanupCommitment(msg->commitment_id);
        }
    }

    void cleanupCommitment(const std::string& commitment_id) {
        auto it = active_commitments_.find(commitment_id);
        if (it == active_commitments_.end()) {
            return;
        }

        ROS_INFO("=== CLEANING UP COMMITMENT ===");
        ROS_INFO("Commitment ID: %s", commitment_id.c_str());

        CommitmentMetadata& metadata = it->second;

        // Unregister all SPARQL conditions
        for (int64_t reg_id : metadata.registration_ids) {
            yggdrasil_interfaces::UnRegisterEventService srv;
            srv.request.registration_id = reg_id;

            if (unregister_event_client_.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO("  [OK] Unregistered condition (ID: %ld)", reg_id);
                }
            }

            condition_to_commitment_.erase(reg_id);
        }

        active_commitments_.erase(it);

        ROS_INFO("✓ Cleanup complete");
        ROS_INFO("==============================");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "commitment_monitor");
    ros::NodeHandle nh;

    ROS_INFO("========================================");
    ROS_INFO("    Commitment Monitor Node");
    ROS_INFO("========================================");

    CommitmentMonitor monitor(nh);

    ros::spin();

    return 0;
}
