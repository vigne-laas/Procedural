#ifndef PROCEDURAL_MEMORY_SERVICE_CLIENT_H
#define PROCEDURAL_MEMORY_SERVICE_CLIENT_H

#include <ros/ros.h>
#include <procedural_interfaces/GetActions.h>
#include <procedural_interfaces/GetTasks.h>
#include <procedural_interfaces/GetActionDetails.h>
#include <procedural_interfaces/GetTaskDetails.h>
#include <procedural_interfaces/Action.h>
#include <procedural_interfaces/Task.h>

namespace procedural {

/**
 * @brief Client wrapper for memory module ROS services
 * Provides simplified interface for accessing procedural memory data
 */
class MemoryServiceClient {
public:
    /**
     * @brief Constructor
     * @param node_handle ROS node handle for service calls
     * @param service_namespace Namespace for memory services (default: "procedural_memory")
     */
    explicit MemoryServiceClient(ros::NodeHandle& node_handle,
                                const std::string& service_namespace = "procedural_memory");

    /**
     * @brief Destructor
     */
    ~MemoryServiceClient() = default;

    /**
     * @brief Get all actions from memory service
     * @param filter Optional filter string (empty for all actions)
     * @return Vector of Action messages or empty vector on failure
     */
    std::vector<procedural_interfaces::Action> getActions(const std::string& filter = "") const;

    /**
     * @brief Get all tasks from memory service
     * @param filter Optional filter string (empty for all tasks)
     * @return Vector of Task messages or empty vector on failure
     */
    std::vector<procedural_interfaces::Task> getTasks(const std::string& filter = "") const;

    /**
     * @brief Get detailed information for a specific action
     * @param action_name Name of the action
     * @return Action details or empty Action on failure
     */
    procedural_interfaces::Action getActionDetails(const std::string& action_name) const;

    /**
     * @brief Get detailed information for a specific task
     * @param task_name Name of the task
     * @return Task details or empty Task on failure
     */
    procedural_interfaces::Task getTaskDetails(const std::string& task_name) const;

    /**
     * @brief Check if memory service is available
     * @return true if service is reachable, false otherwise
     */
    bool isServiceAvailable() const;

    /**
     * @brief Wait for memory service to become available
     * @param timeout Maximum time to wait (default: 30 seconds)
     * @return true if service becomes available, false on timeout
     */
    bool waitForService(const ros::Duration& timeout = ros::Duration(30.0)) const;

private:
    ros::NodeHandle& nh_;
    std::string service_namespace_;

    // Service clients
    mutable ros::ServiceClient get_actions_client_;
    mutable ros::ServiceClient get_tasks_client_;
    mutable ros::ServiceClient get_action_details_client_;
    mutable ros::ServiceClient get_task_details_client_;

    /**
     * @brief Initialize service clients
     */
    void initializeClients();

    /**
     * @brief Ensure service clients are connected
     */
    void ensureClientsConnected() const;

    /**
     * @brief Log service call error
     * @param service_name Name of the failed service
     * @param error_msg Error message
     */
    void logServiceError(const std::string& service_name, const std::string& error_msg) const;
};

} // namespace procedural

#endif // PROCEDURAL_MEMORY_SERVICE_CLIENT_H