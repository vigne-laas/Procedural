#include "procedural/task_recognition/Reader/MemoryDomainReader.h"
#include "procedural/utils/Logger.h"

namespace procedural {

MemoryDomainReader::MemoryDomainReader(ros::NodeHandle& node_handle,
                                     const std::string& service_namespace)
    : nh_(node_handle) {
    memory_client_ = std::make_unique<MemoryServiceClient>(nh_, service_namespace);
}

MemoryDomainReader::MemoryDomainReader(ros::NodeHandle& node_handle, const std::string& filter,
                                     const std::string& service_namespace)
    : nh_(node_handle) {
    memory_client_ = std::make_unique<MemoryServiceClient>(nh_, service_namespace);
    read(filter);
}

bool MemoryDomainReader::read(const std::string& filter) {
    htn_ = HTNParserd_t(); // Clear previous data

    if (!memory_client_->isServiceAvailable()) {
        LOG_ERROR << "Memory service is not available";
        return false;
    }

    try {
        // Fetch tasks from memory service
        auto ros_tasks = memory_client_->getTasks(filter);

        if (ros_tasks.empty()) {
            LOG_WARNING << "No tasks retrieved from memory service";
            return false;
        }

        // Process and convert tasks
        processTasks(ros_tasks);

        LOG_INFO << "Successfully loaded " << htn_.tasks.size() << " abstract tasks and "
                 << htn_.actions.size() << " primitive actions from memory";

        return hasHTN();

    } catch (const std::exception& e) {
        LOG_ERROR << "Exception while reading tasks from memory: " << e.what();
        return false;
    }
}

const HTNParserd_t& MemoryDomainReader::getHTN() const {
    return htn_;
}

bool MemoryDomainReader::hasHTN() const {
    return !htn_.empty();
}

size_t MemoryDomainReader::getTaskCount() const {
    return htn_.tasks.size() + htn_.actions.size();
}

bool MemoryDomainReader::waitForService(const ros::Duration& timeout) {
    return memory_client_->waitForService(timeout);
}

bool MemoryDomainReader::isServiceAvailable() const {
    return memory_client_->isServiceAvailable();
}

void MemoryDomainReader::processTasks(const std::vector<procedural_interfaces::Task>& ros_tasks) {
    // Use the converter to transform ROS messages to internal HTN structure
    htn_ = RosToInternalConverter::convertTasksToHTN(ros_tasks);

    LOG_INFO << "Processed " << ros_tasks.size() << " ROS tasks into HTN with "
             << htn_.tasks.size() << " abstract tasks and "
             << htn_.actions.size() << " primitive actions";
}

} // namespace procedural