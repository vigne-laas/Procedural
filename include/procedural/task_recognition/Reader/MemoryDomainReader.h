#ifndef PROCEDURAL_MEMORY_DOMAIN_READER_H
#define PROCEDURAL_MEMORY_DOMAIN_READER_H

#include "procedural/memory_client/MemoryServiceClient.h"
#include "procedural/memory_client/RosToInternalConverter.h"
#include "procedural/task_recognition/Reader/domainTypes/ParsedHTN.h"
#include <ros/ros.h>

namespace procedural {

/**
 * @brief Memory-based reader for task recognition domain data
 *
 * This class replaces DomainReader by fetching task and HTN definitions from the memory service
 * instead of reading domain files. It provides the same interface but uses ROS services
 * to retrieve task data.
 */
class MemoryDomainReader {
public:
    /**
     * @brief Constructor
     * @param node_handle ROS node handle for service communication
     * @param service_namespace Namespace for memory services (default: "procedural_memory")
     */
    explicit MemoryDomainReader(ros::NodeHandle& node_handle,
                               const std::string& service_namespace = "procedural_memory");

    /**
     * @brief Constructor with automatic read
     * @param node_handle ROS node handle for service communication
     * @param filter Task filter to apply when reading
     * @param service_namespace Namespace for memory services
     */
    MemoryDomainReader(ros::NodeHandle& node_handle, const std::string& filter,
                      const std::string& service_namespace);

    /**
     * @brief Destructor
     */
    ~MemoryDomainReader() = default;

    /**
     * @brief Read tasks and HTN from memory service
     * @param filter Optional filter for task names (empty for all tasks)
     * @return true if tasks were successfully retrieved, false otherwise
     */
    bool read(const std::string& filter = "");

    /**
     * @brief Get the parsed HTN structure
     * @return Reference to the HTN structure
     */
    const HTNParserd_t& getHTN() const;

    /**
     * @brief Check if HTN data was loaded
     * @return true if HTN data was loaded, false otherwise
     */
    bool hasHTN() const;

    /**
     * @brief Get total number of tasks loaded
     * @return Total count of abstract tasks + primitive actions
     */
    size_t getTaskCount() const;

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

    HTNParserd_t htn_;

    /**
     * @brief Process and convert ROS tasks to internal HTN structure
     * @param ros_tasks Vector of ROS Task messages
     */
    void processTasks(const std::vector<procedural_interfaces::Task>& ros_tasks);
};

} // namespace procedural

#endif // PROCEDURAL_MEMORY_DOMAIN_READER_H