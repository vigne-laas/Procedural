#ifndef PROCEDURAL_MEMORY_ACTION_READER_H
#define PROCEDURAL_MEMORY_ACTION_READER_H

#include "procedural/memory_client/MemoryServiceClient.h"
#include "procedural/memory_client/RosToInternalConverter.h"
#include "procedural/action_recognition/reader/types/ParsedSimpleAction.h"
#include "procedural/action_recognition/reader/types/ParsedComposedAction.h"
#include <ros/ros.h>

namespace procedural {

/**
 * @brief Memory-based reader for action recognition
 *
 * This class replaces YamlReader by fetching action definitions from the memory service
 * instead of reading YAML files. It provides the same interface but uses ROS services
 * to retrieve action data.
 */
class MemoryActionReader {
public:
    /**
     * @brief Constructor
     * @param node_handle ROS node handle for service communication
     * @param service_namespace Namespace for memory services (default: "procedural_memory")
     */
    explicit MemoryActionReader(ros::NodeHandle& node_handle,
                               const std::string& service_namespace = "procedural_memory");

    /**
     * @brief Destructor
     */
    ~MemoryActionReader() = default;

    /**
     * @brief Read actions from memory service
     * @param filter Optional filter for action names (empty for all actions)
     * @return true if actions were successfully retrieved, false otherwise
     */
    bool read(const std::string& filter = "");

    /**
     * @brief Get simple actions
     * @return Vector of parsed simple actions
     */
    const std::vector<ParsedSimpleAction_t>& getSimpleActions() const;

    /**
     * @brief Get composed actions
     * @return Vector of parsed composed actions
     */
    const std::vector<ParsedComposedAction_t>& getComposedActions() const;

    /**
     * @brief Check if any actions were loaded
     * @return true if actions were loaded, false otherwise
     */
    bool hasActions() const;

    /**
     * @brief Get total number of actions loaded
     * @return Total count of simple + composed actions
     */
    size_t getActionCount() const;

    /**
     * @brief Wait for memory service to be available
     * @param timeout Maximum time to wait (default: 30 seconds)
     * @return true if service becomes available, false on timeout
     */
    bool waitForService(const ros::Duration& timeout = ros::Duration(30.0));

    /**
     * @brief Check if memory service is currently available
     * @return true if service is available, false otherwise
     */
    bool isServiceAvailable() const;

private:
    ros::NodeHandle& nh_;
    std::unique_ptr<MemoryServiceClient> memory_client_;

    std::vector<ParsedSimpleAction_t> simple_actions_;
    std::vector<ParsedComposedAction_t> composed_actions_;

    /**
     * @brief Process and convert ROS actions to internal structures
     * @param ros_actions Vector of ROS Action messages
     */
    void processActions(const std::vector<procedural_interfaces::Action>& ros_actions);
};

} // namespace procedural

#endif // PROCEDURAL_MEMORY_ACTION_READER_H