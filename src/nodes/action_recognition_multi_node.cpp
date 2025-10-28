#include <ros/ros.h>
#include <signal.h>
#include <thread>
#include <map>
#include <regex>
#include <std_msgs/String.h>

#include "procedural/ActionRecognitionNode.h"
#include "ontologenius/OntologiesManipulator.h"
#include "mementar/TimelinesManipulator.h"
#include "overworld/GetAgents.h"

#include "procedural/utils/Logger.h"

// Messages
#include "procedural_interfaces/RecognitionStateChange.h"
#include "procedural_interfaces/RecognizedAction.h"
#include "procedural_interfaces/ActiveGraphsState.h"
#include "procedural_interfaces/Description.h"
#include "procedural_interfaces/StateMachineLibrary.h"

using namespace procedural;

/**
 * MultiNodeManager
 *
 * Manages multiple ActionRecognitionNode instances, one per agent/timeline.
 * Centralizes publication of all events and aggregates state from all timelines.
 */
class MultiNodeManager {
public:
    MultiNodeManager(ros::NodeHandle* nh)
        : nh_(nh)
    {
        LOG_INFO << "=== Multi-Agent Action Recognition Manager Starting ===";

        // Initialize global ontology and timeline manipulators
        onto_manipulators_ = new onto::OntologiesManipulator();
        time_manipulators_ = new mementar::TimelinesManipulator(nh_);

        LOG_INFO << "Ontology and Timeline manipulators created";

        // Initialize centralized publishers
        initializePublishers();

        // Subscribe to overworld for agent management
        new_agent_sub_ = nh_->subscribe("/overworld/new_assessor", 10,
                                       &MultiNodeManager::onNewAgent, this);

        get_agents_client_ = nh_->serviceClient<overworld::GetAgents>("/overworld/getAgents");

        LOG_INFO << "Subscribed to /overworld/new_assessor for new agent notifications";

        // Query existing agents from overworld
        queryExistingAgents();

        LOG_INFO << "=== Multi-Agent Action Recognition Manager Ready ===";
    }

    ~MultiNodeManager() {
        // Stop all nodes
        for (auto& pair : nodes_) {
            pair.second->stop();
        }

        // Join all threads
        for (auto& pair : threads_) {
            if (pair.second.joinable()) {
                pair.second.join();
            }
        }

        // Delete nodes
        for (auto& pair : nodes_) {
            delete pair.second;
        }

        // Delete manipulators
        delete onto_manipulators_;
        delete time_manipulators_;

        LOG_INFO << "Multi-Agent Action Recognition Manager shutdown complete";
    }

    void spin() {
        ros::spin();
    }

private:
    void initializePublishers() {
        // Centralized publishers for ALL timelines
        state_change_pub_ = nh_->advertise<procedural_interfaces::RecognitionStateChange>(
            "/recognition/state_changes", 100, true);
        recognized_action_pub_ = nh_->advertise<procedural_interfaces::RecognizedAction>(
            "/recognition/actions", 100, true);
        description_pub_ = nh_->advertise<procedural_interfaces::Description>(
            "/recognition/descriptions", 100, true);
        description_str_pub_ = nh_->advertise<std_msgs::String>(
            "/recognition/descriptions_str", 100, true);
        active_graphs_state_pub_ = nh_->advertise<procedural_interfaces::ActiveGraphsState>(
            "/recognition/active_graphs_state", 100, true);
        library_pub_ = nh_->advertise<procedural_interfaces::StateMachineLibrary>(
            "/recognition/library", 10, true);

        LOG_INFO << "Centralized publishers initialized:";
        LOG_INFO << "  - /recognition/state_changes";
        LOG_INFO << "  - /recognition/actions";
        LOG_INFO << "  - /recognition/descriptions";
        LOG_INFO << "  - /recognition/descriptions_str";
        LOG_INFO << "  - /recognition/active_graphs_state";
        LOG_INFO << "  - /recognition/library";
    }

    void queryExistingAgents() {
        LOG_INFO << "Querying existing agents from overworld...";

        if (!get_agents_client_.waitForExistence(ros::Duration(5.0))) {
            ROS_WARN("Service /overworld/getAgents not available, skipping existing agents query");
            return;
        }

        overworld::GetAgents srv;
        if (get_agents_client_.call(srv)) {
            LOG_INFO << "Found " << srv.response.agents.size() << " existing agents";
            for (const auto& agent : srv.response.agents) {
                LOG_INFO << "  - Creating node for agent: " << agent;
                createNodeForAgent(agent);
            }
        } else {
            LOG_ERROR << "Failed to call /overworld/getAgents service";
        }
    }

    void onNewAgent(const std_msgs::String::ConstPtr& msg) {
        try {
            LOG_DEBUG << "Received message on /overworld/new_assessor: '" << msg->data << "'";

            // Parse agent name from message format: "ADD|agent_name"
            auto parts = parseMessage(msg->data);

            if (parts.size() >= 2 && parts[0] == "ADD") {
                std::string agent_name = parts[1];
                LOG_INFO << "New agent detected: " << agent_name;
                createNodeForAgent(agent_name);
            } else if (parts.empty()) {
                ROS_WARN("Received empty message on /overworld/new_assessor");
            } else {
                ROS_WARN("Received invalid message format on /overworld/new_assessor: '%s' (expected 'ADD|agent_name')",
                         msg->data.c_str());
            }
        } catch (const std::exception& e) {
            LOG_ERROR << "Exception in onNewAgent callback: " << e.what()
                      << " (message was: '" << msg->data << "')";
        }
    }

    std::vector<std::string> parseMessage(const std::string& msg) {
        std::regex regex("\\|");
        std::vector<std::string> out(
            std::sregex_token_iterator(msg.begin(), msg.end(), regex, -1),
            std::sregex_token_iterator()
        );
        return out;
    }

    void createNodeForAgent(const std::string& agent_name) {
        // Check if node already exists
        if (nodes_.find(agent_name) != nodes_.end()) {
            ROS_WARN("Node for agent %s already exists, skipping", agent_name.c_str());
            return;
        }

        try {
            LOG_INFO << "=== Creating Action Recognition Node for: " << agent_name << " ===";

            // Create dedicated ontology for this agent
            onto_manipulators_->waitInit();
            onto_manipulators_->add(agent_name);
            auto onto_manipulator = onto_manipulators_->get(agent_name);

            LOG_INFO << "[" << agent_name << "] Dedicated ontology created";

            // Create dedicated timeline for this agent
            time_manipulators_->waitInit();
            time_manipulators_->add(agent_name);
            auto timeline_manipulator = time_manipulators_->get(agent_name);

            LOG_INFO << "[" << agent_name << "] Dedicated timeline created";

            // Create ActionRecognitionNode instance
            auto node = new ActionRecognitionNode(nh_, onto_manipulator, timeline_manipulator, agent_name);

            // Set up callbacks for centralized publication
            node->setRecognizedActionCallback([this](const procedural_interfaces::RecognizedAction& msg) {
                onRecognizedAction(msg);
            });

            node->setStateChangeCallback([this](const procedural_interfaces::RecognitionStateChange& msg) {
                onStateChange(msg);
            });

            node->setActiveGraphsCallback([this](const std::vector<Graph*>& graphs, const std::string& timeline_id) {
                onActiveGraphsUpdate(graphs, timeline_id);
            });

            node->setDescriptionCallback([this](const procedural_interfaces::Description& msg) {
                onDescription(msg);
            });

            // Initialize the node
            if (!node->initializeActionRecognition()) {
                LOG_ERROR << "Failed to initialize Action Recognition for agent: " << agent_name;
                delete node;
                return;
            }

            // Store node
            nodes_[agent_name] = node;

            // Launch thread for this node
            threads_[agent_name] = std::thread(&ActionRecognitionNode::spin, node);

            LOG_INFO << "[" << agent_name << "] Action Recognition Node STARTED";
            LOG_INFO << "=== Total active timelines: " << nodes_.size() << " ===";

            // Publish aggregated library after adding a new agent
            publishAggregatedLibrary();

        } catch (const std::exception& e) {
            LOG_ERROR << "Exception while creating node for agent '" << agent_name << "': " << e.what();
            LOG_ERROR << "Agent '" << agent_name << "' will NOT be tracked for action recognition";
        }
    }

    // Callback handlers - Centralized publication
    void onRecognizedAction(const procedural_interfaces::RecognizedAction& msg) {
        LOG_INFO << "[" << msg.timeline_id << "] Publishing recognized action: " << msg.action_name;
        recognized_action_pub_.publish(msg);
    }

    void onStateChange(const procedural_interfaces::RecognitionStateChange& msg) {
        LOG_INFO << "[" << msg.timeline_id << "] Publishing state change: " << msg.event_type;
        state_change_pub_.publish(msg);
    }

    void onDescription(const procedural_interfaces::Description& msg) {
        LOG_INFO << "Publishing " << msg.descriptions.size() << " descriptions";
        description_pub_.publish(msg);

        // Also publish string format
        for (const auto& triplet : msg.descriptions) {
            std_msgs::String str_msg;
            str_msg.data = "[ADD]" + trim(triplet.subject.literal) + "|"
                                   + trim(triplet.property) + "|"
                                   + trim(triplet.object.literal);
            description_str_pub_.publish(str_msg);
        }
    }

    void onActiveGraphsUpdate(const std::vector<Graph*>& graphs, const std::string& timeline_id) {
        LOG_INFO << "[" << timeline_id << "] Active graphs update received (" << graphs.size() << " graphs)";

        // Update internal state for this timeline
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_graphs_per_timeline_[timeline_id] = graphs;
        }

        // Publish COMPLETE state of ALL timelines (aggregated)
        publishCompleteState();
    }

    void publishCompleteState() {
        std::lock_guard<std::mutex> lock(state_mutex_);

        procedural_interfaces::ActiveGraphsState global_state;
        global_state.header.stamp = ros::Time::now();

        int total_graphs = 0;

        // Aggregate graphs from ALL timelines
        for (const auto& [timeline_id, graphs] : active_graphs_per_timeline_) {
            global_state.timeline_ids.push_back(timeline_id);

            for (const auto& graph : graphs) {
                // Get the node to convert graph to message using public method
                auto node_it = nodes_.find(timeline_id);
                if (node_it != nodes_.end()) {
                    auto graph_msg = node_it->second->convertGraphToMsg(graph);
                    // Ensure timeline_id is set (already done in convertGraphToMsg but double-check)
                    graph_msg.timeline_id = timeline_id;

                    global_state.active_graphs.push_back(graph_msg);
                    total_graphs++;
                }
            }
        }

        LOG_INFO << "Publishing COMPLETE state: " << global_state.timeline_ids.size()
                 << " timelines, " << total_graphs << " total graphs";

        active_graphs_state_pub_.publish(global_state);
    }

    void publishAggregatedLibrary() {
        procedural_interfaces::StateMachineLibrary lib_msg;

        int total_count = 0;
        int active_count = 0;
        int completed_count = 0;

        // Aggregate library data from all nodes
        for (const auto& [agent_name, node] : nodes_) {
            if (!node) continue;

            auto* action_builder = node->getActionBuilder();
            if (!action_builder) continue;

            auto actions = action_builder->getActions();

            // actions is a std::vector<Action*>
            for (auto* action : actions) {
                if (!action) continue;

                // Get the factory graph from the action
                auto* factory = action->getFactory();
                if (!factory) continue;

                // Convert Graph to StateMachineGraph message
                auto graph_msg = node->convertGraphToMsg(factory);

                // Set timeline_id to agent_name for multi-agent tracking
                graph_msg.timeline_id = agent_name;

                lib_msg.graphs.push_back(graph_msg);
                total_count++;

                // Count active/completed based on graph state
                // (for now, all are considered as part of the library)
            }
        }

        lib_msg.total_count = total_count;
        lib_msg.active_count = active_count;
        lib_msg.completed_count = completed_count;

        LOG_INFO << "Publishing aggregated library: " << total_count << " graphs from "
                 << nodes_.size() << " agents";

        library_pub_.publish(lib_msg);
    }

    // Helper function to trim whitespace
    static std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\n\r");
        return str.substr(first, last - first + 1);
    }

    // ROS interface
    ros::NodeHandle* nh_;
    ros::Subscriber new_agent_sub_;
    ros::ServiceClient get_agents_client_;

    // Centralized publishers
    ros::Publisher state_change_pub_;
    ros::Publisher recognized_action_pub_;
    ros::Publisher description_pub_;
    ros::Publisher description_str_pub_;
    ros::Publisher active_graphs_state_pub_;
    ros::Publisher library_pub_;

    // Ontology and timeline management
    onto::OntologiesManipulator* onto_manipulators_;
    mementar::TimelinesManipulator* time_manipulators_;

    // Node management
    std::map<std::string, ActionRecognitionNode*> nodes_;
    std::map<std::string, std::thread> threads_;

    // Aggregated state from all timelines
    std::map<std::string, std::vector<Graph*>> active_graphs_per_timeline_;
    std::mutex state_mutex_;  // Protect shared state
};

// Signal handler
void signalHandler(int sig) {
    LOG_INFO << "Shutting down multi-agent action recognition...";
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_recognition_multi");

    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ros::NodeHandle nh;

    try {
        MultiNodeManager manager(&nh);
        manager.spin();
    } catch (const std::exception& e) {
        LOG_ERROR << "Multi-agent action recognition failed: " << e.what();
        return 1;
    }

    return 0;
}
