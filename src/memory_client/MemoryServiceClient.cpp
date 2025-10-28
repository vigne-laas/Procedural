#include "procedural/memory_client/MemoryServiceClient.h"
#include "procedural/utils/Logger.h"

namespace procedural {

MemoryServiceClient::MemoryServiceClient(ros::NodeHandle& node_handle,
                                        const std::string& service_namespace)
    : nh_(node_handle), service_namespace_(service_namespace) {
    initializeClients();
}

void MemoryServiceClient::initializeClients() {
    // Memory services are exposed globally without namespace
    // Check if namespace is provided, if empty use global services
    if (service_namespace_.empty() || service_namespace_ == "procedural_memory") {
        // Use global services (actual service names from MemoryRosInterface)
        get_actions_client_ = nh_.serviceClient<procedural_interfaces::GetActions>("/getActions");
        get_tasks_client_ = nh_.serviceClient<procedural_interfaces::GetTasks>("/getTasks");
        get_action_details_client_ = nh_.serviceClient<procedural_interfaces::GetActionDetails>("/getActionDetails");
        get_task_details_client_ = nh_.serviceClient<procedural_interfaces::GetTaskDetails>("/getTaskDetails");
    } else {
        // Use namespaced services
        std::string base_namespace = service_namespace_;
        if (!base_namespace.empty() && base_namespace.back() != '/') {
            base_namespace += "/";
        }
        get_actions_client_ = nh_.serviceClient<procedural_interfaces::GetActions>(
            base_namespace + "getActions");
        get_tasks_client_ = nh_.serviceClient<procedural_interfaces::GetTasks>(
            base_namespace + "getTasks");
        get_action_details_client_ = nh_.serviceClient<procedural_interfaces::GetActionDetails>(
            base_namespace + "getActionDetails");
        get_task_details_client_ = nh_.serviceClient<procedural_interfaces::GetTaskDetails>(
            base_namespace + "getTaskDetails");
    }
}

void MemoryServiceClient::ensureClientsConnected() const {
    // Recreate clients using the same logic as initializeClients
    if (service_namespace_.empty() || service_namespace_ == "procedural_memory") {
        // Use global services
        if (!get_actions_client_.isValid()) {
            get_actions_client_ = nh_.serviceClient<procedural_interfaces::GetActions>("/getActions");
        }
        if (!get_tasks_client_.isValid()) {
            get_tasks_client_ = nh_.serviceClient<procedural_interfaces::GetTasks>("/getTasks");
        }
        if (!get_action_details_client_.isValid()) {
            get_action_details_client_ = nh_.serviceClient<procedural_interfaces::GetActionDetails>("/getActionDetails");
        }
        if (!get_task_details_client_.isValid()) {
            get_task_details_client_ = nh_.serviceClient<procedural_interfaces::GetTaskDetails>("/getTaskDetails");
        }
    } else {
        // Use namespaced services
        if (!get_actions_client_.isValid()) {
            get_actions_client_ = nh_.serviceClient<procedural_interfaces::GetActions>(
                service_namespace_ + "/getActions");
        }
        if (!get_tasks_client_.isValid()) {
            get_tasks_client_ = nh_.serviceClient<procedural_interfaces::GetTasks>(
                service_namespace_ + "/getTasks");
        }
        if (!get_action_details_client_.isValid()) {
            get_action_details_client_ = nh_.serviceClient<procedural_interfaces::GetActionDetails>(
                service_namespace_ + "/getActionDetails");
        }
        if (!get_task_details_client_.isValid()) {
            get_task_details_client_ = nh_.serviceClient<procedural_interfaces::GetTaskDetails>(
                service_namespace_ + "/getTaskDetails");
        }
    }
}

std::vector<procedural_interfaces::Action> MemoryServiceClient::getActions(const std::string& filter) const {
    ensureClientsConnected();

    procedural_interfaces::GetActions srv;
    srv.request.filter = filter;

    if (get_actions_client_.call(srv)) {
        LOG_INFO << "Successfully retrieved " << srv.response.actions.size() << " actions from memory service";
        return srv.response.actions;
    } else {
        logServiceError("GetActions", "Failed to call service");
        return {};
    }
}

std::vector<procedural_interfaces::Task> MemoryServiceClient::getTasks(const std::string& filter) const {
    ensureClientsConnected();

    procedural_interfaces::GetTasks srv;
    srv.request.filter = filter;

    if (get_tasks_client_.call(srv)) {
        LOG_INFO << "Successfully retrieved " << srv.response.tasks.tasks.size() << " tasks from memory service";
        return srv.response.tasks.tasks;
    } else {
        logServiceError("GetTasks", "Failed to call service");
        return {};
    }
}

procedural_interfaces::Action MemoryServiceClient::getActionDetails(const std::string& action_name) const {
    ensureClientsConnected();

    procedural_interfaces::GetActionDetails srv;
    srv.request.action_name = action_name;

    if (get_action_details_client_.call(srv)) {
        LOG_INFO << "Successfully retrieved details for action: " << action_name;
        return srv.response.action;
    } else {
        logServiceError("GetActionDetails", "Failed to get details for action: " + action_name);
        return {};
    }
}

procedural_interfaces::Task MemoryServiceClient::getTaskDetails(const std::string& task_name) const {
    ensureClientsConnected();

    procedural_interfaces::GetTaskDetails srv;
    srv.request.task_name = task_name;

    if (get_task_details_client_.call(srv)) {
        LOG_INFO << "Successfully retrieved details for task: " << task_name;
        return srv.response.task;
    } else {
        logServiceError("GetTaskDetails", "Failed to get details for task: " + task_name);
        return {};
    }
}

bool MemoryServiceClient::isServiceAvailable() const {
    ensureClientsConnected();

    return get_actions_client_.exists() &&
           get_tasks_client_.exists() &&
           get_action_details_client_.exists() &&
           get_task_details_client_.exists();
}

bool MemoryServiceClient::waitForService(const ros::Duration& timeout) const {
    ensureClientsConnected();

    ros::Time start_time = ros::Time::now();
    ros::Duration poll_duration(0.1); // Poll every 100ms

    while (ros::Time::now() - start_time < timeout) {
        if (isServiceAvailable()) {
            LOG_INFO << "Memory services are available";
            return true;
        }

        poll_duration.sleep();
        ros::spinOnce();
    }

    LOG_ERROR << "Timeout waiting for memory services to become available";
    return false;
}

void MemoryServiceClient::logServiceError(const std::string& service_name, const std::string& error_msg) const {
    LOG_ERROR << "MemoryServiceClient::" << service_name << " - " << error_msg;
}

} // namespace procedural