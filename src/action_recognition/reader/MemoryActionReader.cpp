#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural/utils/Logger.h"

namespace procedural {

MemoryActionReader::MemoryActionReader(ros::NodeHandle& node_handle,
                                     const std::string& service_namespace)
    : nh_(node_handle) {
    memory_client_ = std::make_unique<MemoryServiceClient>(nh_, service_namespace);
}

bool MemoryActionReader::read(const std::string& filter) {
    simple_actions_.clear();
    composed_actions_.clear();

    if (!memory_client_->isServiceAvailable()) {
        LOG_ERROR << "Memory service is not available";
        return false;
    }

    try {
        // Fetch actions from memory service
        auto ros_actions = memory_client_->getActions(filter);

        if (ros_actions.empty()) {
            LOG_WARNING << "No actions retrieved from memory service";
            return false;
        }

        // Process and convert actions
        processActions(ros_actions);

        LOG_INFO << "Successfully loaded " << simple_actions_.size() << " simple actions and "
                 << composed_actions_.size() << " composed actions from memory";

        return hasActions();

    } catch (const std::exception& e) {
        LOG_ERROR << "Exception while reading actions from memory: " << e.what();
        return false;
    }
}

const std::vector<ParsedSimpleAction_t>& MemoryActionReader::getSimpleActions() const {
    return simple_actions_;
}

const std::vector<ParsedComposedAction_t>& MemoryActionReader::getComposedActions() const {
    return composed_actions_;
}

bool MemoryActionReader::hasActions() const {
    return !simple_actions_.empty() || !composed_actions_.empty();
}

size_t MemoryActionReader::getActionCount() const {
    return simple_actions_.size() + composed_actions_.size();
}

bool MemoryActionReader::waitForService(const ros::Duration& timeout) {
    return memory_client_->waitForService(timeout);
}

bool MemoryActionReader::isServiceAvailable() const {
    return memory_client_->isServiceAvailable();
}

void MemoryActionReader::processActions(const std::vector<procedural_interfaces::Action>& ros_actions) {
    LOG_INFO << "Processing " << ros_actions.size() << " actions from memory service";

    // Log each action received with recognition sequence info
    for (const auto& action : ros_actions) {
        LOG_INFO << "  - Action: " << action.actionName
                 << " (arguments: " << action.arguments.size()
                 << ", preconditions: " << action.preconditions.size()
                 << ", effects: " << action.effects.size()
                 << ", recognition facts: " << action.recognition.sequence.size() << ")";
    }

    // Use the converter to transform ROS messages to internal structures
    size_t initial_count = ros_actions.size();
    RosToInternalConverter::convertActionsToInternal(ros_actions, simple_actions_, composed_actions_);
    size_t converted_count = simple_actions_.size() + composed_actions_.size();

    LOG_INFO << "Processed " << ros_actions.size() << " ROS actions into "
             << simple_actions_.size() << " simple and "
             << composed_actions_.size() << " composed actions";

    if (converted_count < initial_count) {
        LOG_INFO << "Note: " << (initial_count - converted_count)
                 << " action(s) were filtered out (no recognition sequence)";
    }

    // Log details of converted actions
    for (const auto& simple_action : simple_actions_) {
        LOG_INFO << "  Simple action: " << simple_action.getName();
    }
    for (const auto& composed_action : composed_actions_) {
        LOG_INFO << "  Composed action: " << composed_action.getName();
    }
}

} // namespace procedural