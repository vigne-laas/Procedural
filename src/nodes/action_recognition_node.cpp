#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <mementar/StampedFact.h>

#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural/action_recognition/builder/ActionBuilder.h"
#include "procedural/action_recognition/core/ActionRecognition.h"
#include "procedural/utils/Logger.h"
#include "procedural/utils/TimeStamp.h"
#include "procedural/structures/Fact.h"
#include "procedural/structures/ObservationFact.h"

// New messages for introspection
#include "procedural_interfaces/StateMachineNode.h"
#include "procedural_interfaces/StateMachineTransition.h"
#include "procedural_interfaces/StateMachineGraph.h"
#include "procedural_interfaces/StateMachineLibrary.h"
#include "procedural_interfaces/RecognitionStateChange.h"
#include "procedural_interfaces/RecognizedAction.h"
#include "procedural_interfaces/GetCurrentState.h"
#include "procedural_interfaces/GetGraphDetails.h"
#include "procedural_interfaces/ResetRecognition.h"
#include "procedural_interfaces/ActiveGraphsState.h"

using namespace procedural;

// Helper function to trim whitespace
std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, last - first + 1);
}

class ActionRecognitionNode {
public:
    ActionRecognitionNode() : nh_("~"), action_recognition_(nullptr), action_builder_(nullptr) {
        // Initialize parameters
        nh_.param<std::string>("memory_service_namespace", memory_service_namespace_, "procedural_memory");
        nh_.param<std::string>("action_filter", action_filter_, "");
        nh_.param<double>("fact_time_to_live", fact_ttl_, 10.0);
        nh_.param<int>("buffer_max_size", buffer_max_size_, 1000);
        nh_.param<std::string>("fact_topic", fact_topic_, "/facts");
        nh_.param<bool>("debug_mode", debug_mode_, false);

        LOG_INFO << "Action Recognition Node starting with parameters:";
        LOG_INFO << "  Memory service namespace: " << memory_service_namespace_;
        LOG_INFO << "  Action filter: " << (action_filter_.empty() ? "none" : action_filter_);
        LOG_INFO << "  Fact TTL: " << fact_ttl_ << " seconds";
        LOG_INFO << "  Buffer max size: " << buffer_max_size_;
        LOG_INFO << "  Fact topic: " << fact_topic_;
        LOG_INFO << "  Debug mode: " << (debug_mode_ ? "enabled" : "disabled");

        initializeActionRecognition();
        initializeRosInterface();
    }

    ~ActionRecognitionNode() {
        if (action_recognition_) {
            delete action_recognition_;
        }
        if (action_builder_) {
            delete action_builder_;
        }
    }

    bool initializeActionRecognition() {
        try {
            // Create memory reader
            auto memory_reader = std::make_unique<MemoryActionReader>(nh_, memory_service_namespace_);

            // Wait for memory service
            LOG_INFO << "Waiting for memory service to become available...";
            if (!memory_reader->waitForService(ros::Duration(30.0))) {
                LOG_ERROR << "Memory service not available after 30 seconds";
                return false;
            }

            // Read actions from memory
            LOG_INFO << "Reading actions from memory service...";
            if (!memory_reader->read(action_filter_)) {
                LOG_ERROR << "Failed to read actions from memory service";
                return false;
            }

            LOG_INFO << "Successfully loaded " << memory_reader->getActionCount() << " actions";

            // Build actions using ActionBuilder
            LOG_INFO << "Building action state machines...";
            auto simple_actions = memory_reader->getSimpleActions();
            auto composed_actions = memory_reader->getComposedActions();

            action_builder_ = new ActionBuilder(simple_actions, composed_actions, "/tmp/procedural_debug");
            auto actions = action_builder_->getActions();

            if (actions.empty()) {
                LOG_ERROR << "No actions were built successfully";
                return false;
            }

            LOG_INFO << "Successfully built " << actions.size() << " action state machines";

            // Initialize ActionRecognition
            action_recognition_ = new ActionRecognition();
            action_recognition_->init(actions, fact_ttl_, buffer_max_size_);

            // Set up callbacks for different events
            action_recognition_->setCallback([this](const std::vector<Graph*>& graphs) {
                handleRecognizedGraphs(graphs);
            });

            action_recognition_->setActiveGraphsCallback([this](const std::vector<Graph*>& graphs) {
                handleActiveGraphsUpdate(graphs);
            });

            LOG_INFO << "Action recognition system initialized successfully";
            LOG_INFO << "Properties in WordTable:\n" << WordTable::properties_table.toString();
            return true;

        } catch (const std::exception& e) {
            LOG_ERROR << "Exception during initialization: " << e.what();
            return false;
        }
    }

    void handleRecognizedGraphs(const std::vector<Graph*>& graphs) {
        for (const auto& graph : graphs) {
            LOG_INFO << "Recognized action graph: " << graph->getName();

            // Publish recognized action
            procedural_interfaces::RecognizedAction action_msg;
            action_msg.header.stamp = ros::Time::now();
            action_msg.action_name = graph->getName();
            action_msg.action_type = "action";
            action_msg.start_time = ros::Time::now();  // Should track actual start time
            action_msg.end_time = ros::Time::now();
            action_msg.confidence = 1.0;  // Should get actual confidence

            // Extract bound variables from graph
            auto& var_table = graph->getTableVariables();
            for (const auto& var_pair : var_table.variables) {
                if (var_pair.second && var_pair.second->isSet()) {
                    procedural_interfaces::Argument arg;
                    arg.literal = var_pair.first;  // Variable name (e.g., "L")
                    arg.type = var_pair.second->getType();  // Type (e.g., "Location")
                    arg.value = WordTable::individuals_table.get(var_pair.second->getValue());  // Bound value (e.g., "WelcomingArea")
                    action_msg.arguments.push_back(arg);
                    LOG_DEBUG << "  Bound variable: " << arg.literal << " (" << arg.type << ") = " << arg.value;
                }
            }

            recognized_action_pub_.publish(action_msg);

            // Publish descriptions if available
            LOG_DEBUG << "Checking for descriptions for graph: " << graph->getName();
            if (action_builder_) {
                LOG_DEBUG << "  action_builder_ is valid";
                auto actions = action_builder_->getActions();
                LOG_DEBUG << "  Total actions in builder: " << actions.size();
                // Find the action corresponding to this graph
                for (const auto& action : actions) {
                    LOG_DEBUG << "  Comparing action '" << action->getName() << "' with graph '" << graph->getName() << "'";
                    if (action->getName() == graph->getName()) {
                        LOG_DEBUG << "  Found matching action!";
                        auto descriptions = action->getDescriptions();
                        LOG_DEBUG << "  Number of descriptions: " << descriptions.descriptions.size();
                        if (!descriptions.descriptions.empty()) {
                            procedural_interfaces::Description desc_msg;

                            for (const auto& parsed_desc : descriptions.descriptions) {
                                procedural_interfaces::Triplet triplet;
                                triplet.add = true;  // Descriptions are assertions
                                triplet.required = false;

                                // Replace variables in subject, property, object
                                LOG_DEBUG << "  Before replacement - subject: " << parsed_desc.subject
                                         << ", property: " << parsed_desc.property
                                         << ", object: " << parsed_desc.object;
                                triplet.subject.literal = replaceVariables(parsed_desc.subject, var_table, graph);
                                triplet.property = replaceVariables(parsed_desc.property, var_table, graph);
                                triplet.object.literal = replaceVariables(parsed_desc.object, var_table, graph);

                                desc_msg.descriptions.push_back(triplet);
                                LOG_DEBUG << "  After replacement - Description: " << triplet.subject.literal << " "
                                         << triplet.property << " " << triplet.object.literal;
                            }

                            LOG_INFO << "Publishing " << desc_msg.descriptions.size() << " descriptions for action " << graph->getName();
                            description_pub_.publish(desc_msg);

                            // Publish string format - one message per description
                            for (const auto& triplet : desc_msg.descriptions) {
                                std_msgs::String str_msg;
                                str_msg.data = "[ADD]" + trim(triplet.subject.literal) + "|"
                                                       + trim(triplet.property) + "|"
                                                       + trim(triplet.object.literal);
                                description_str_pub_.publish(str_msg);
                                LOG_DEBUG << "Published description: " << str_msg.data;
                            }
                            LOG_INFO << "Published " << desc_msg.descriptions.size() << " descriptions in string format";
                        } else {
                            LOG_DEBUG << "  No descriptions to publish (empty)";
                        }
                        break;
                    }
                }
            } else {
                LOG_DEBUG << "  action_builder_ is NULL!";
            }

            // Publish state change
            procedural_interfaces::RecognitionStateChange state_msg;
            state_msg.header.stamp = ros::Time::now();
            state_msg.event_type = "graph_completed";
            state_msg.affected_graph = convertGraphToMsg(graph);
            state_msg.old_state = "Active";
            state_msg.new_state = "Completed";
            state_msg.triggering_fact = last_fact_;

            state_change_pub_.publish(state_msg);
        }
    }

    void handleActiveGraphsUpdate(const std::vector<Graph*>& graphs) {
        // Log active graphs
        for (const auto& graph : graphs) {
            LOG_INFO << "Active graph updated: " << graph->getName()
                     << " completion: " << graph->getCompletionRatio();
        }

        // Publish state change for each graph individually
        for (const auto& graph : graphs) {
            procedural_interfaces::RecognitionStateChange state_msg;
            state_msg.header.stamp = ros::Time::now();
            state_msg.event_type = "graph_progressed";
            state_msg.affected_graph = convertGraphToMsg(graph);
            state_msg.old_state = "Active";
            state_msg.new_state = "Active";
            state_msg.triggering_fact = last_fact_;

            state_change_pub_.publish(state_msg);
        }

        // Publish complete state of ALL graphs (active + just finished)
        procedural_interfaces::ActiveGraphsState active_state_msg;
        active_state_msg.header.stamp = ros::Time::now();
        active_state_msg.last_triggering_fact = last_fact_;

        // Publish all graphs with their actual state (Active, Finished, or Completed)
        // Finished graphs will be published once, then cleaned up and absent from next message
        int active_count = 0;
        int finished_count = 0;
        for (const auto& graph : graphs) {
            active_state_msg.active_graphs.push_back(convertGraphToMsg(graph));
            if (graph->getState() >= GraphState::Finished) {
                finished_count++;
                LOG_DEBUG << "Publishing finished/completed graph: " << graph->getName()
                         << " (state: " << static_cast<int>(graph->getState()) << ")";
            } else {
                active_count++;
            }
        }

        active_graphs_state_pub_.publish(active_state_msg);
        LOG_INFO << "Published active graphs state with " << active_count
                 << " active graphs and " << finished_count << " finished/completed graphs";
    }

    void spin() {
        if (!action_recognition_) {
            LOG_ERROR << "Action recognition not initialized, cannot spin";
            return;
        }

        ros::Rate rate(10); // 10 Hz

        while (ros::ok()) {
            // Process any incoming ROS messages
            ros::spinOnce();

            // Process the fact queue
            action_recognition_->processQueue(TimeStamp_t(ros::Time::now().toSec()));

            rate.sleep();
        }
    }

    // Initialize ROS interface (subscribers, publishers, services)
    void initializeRosInterface() {
        // Subscriber for facts
        fact_sub_ = nh_.subscribe(fact_topic_, buffer_max_size_,
                                  &ActionRecognitionNode::factCallback, this);

        // Publishers for events (all latched so new subscribers get last message)
        state_change_pub_ = nh_.advertise<procedural_interfaces::RecognitionStateChange>(
            "/recognition/state_changes", 100, true);
        recognized_action_pub_ = nh_.advertise<procedural_interfaces::RecognizedAction>(
            "/recognition/actions", 100, true);
        description_pub_ = nh_.advertise<procedural_interfaces::Description>(
            "/recognition/descriptions", 100, true);
        description_str_pub_ = nh_.advertise<std_msgs::String>(
            "/recognition/descriptions_str", 100, true);
        graph_library_pub_ = nh_.advertise<procedural_interfaces::StateMachineLibrary>(
            "/recognition/library", 10, true);
        active_graphs_state_pub_ = nh_.advertise<procedural_interfaces::ActiveGraphsState>(
            "/recognition/active_graphs_state", 100, true);

        // Services for queries
        get_current_state_srv_ = nh_.advertiseService("/recognition/get_current_state",
            &ActionRecognitionNode::getCurrentStateCallback, this);
        get_graph_details_srv_ = nh_.advertiseService("/recognition/get_graph_details",
            &ActionRecognitionNode::getGraphDetailsCallback, this);
        reset_recognition_srv_ = nh_.advertiseService("/recognition/reset",
            &ActionRecognitionNode::resetRecognitionCallback, this);

        LOG_INFO << "ROS interface initialized";
        LOG_INFO << "  Fact subscriber: " << fact_topic_;
        LOG_INFO << "  Publishers: /recognition/state_changes, /recognition/actions, /recognition/library, /recognition/active_graphs_state";
        LOG_INFO << "  Services: /recognition/get_current_state, /recognition/get_graph_details, /recognition/reset";

        // Publish initial library
        publishInitialLibrary();
    }

    // Convert Variable to ROS message
    procedural_interfaces::Variable convertVariableToMsg(const std::string& name, const std::shared_ptr<Variable_t>& var) {
        procedural_interfaces::Variable var_msg;
        var_msg.literal = name;
        var_msg.type = var->getType();
        var_msg.isVariable = !var->isSet();
        var_msg.isClass = false;
        var_msg.value = var->isSet() ? WordTable::individuals_table[var->getValue()] : "";
        return var_msg;
    }

    // Callback for incoming facts
    void factCallback(const mementar::StampedFact::ConstPtr& msg) {
        try {
            // Log raw ROS message
            LOG_INFO << "========== RECEIVED ROS FACT ==========";
            LOG_INFO << "  added: " << (msg->added ? "true" : "false");
            LOG_INFO << "  subject: '" << msg->subject << "'";
            LOG_INFO << "  predicat: '" << msg->predicat << "'";
            LOG_INFO << "  object: '" << msg->object << "'";

            // Use the 7-parameter constructor with types
            // Use ros::Time::now() instead of msg->stamp to avoid timestamp issues when publishing manually
            Fact* fact = new Fact(msg->added, msg->subject, "unknown", msg->predicat,
                                 msg->object, "unknown",
                                 TimeStamp_t(ros::Time::now().toSec()));

            if (fact->isValid()) {
                LOG_INFO << "  Converted fact: " << fact->toString();
                action_recognition_->addToQueue(fact);
                last_fact_ = fact->toString();
                LOG_INFO << "  ✓ Added to queue";
            } else {
                LOG_ERROR << "  ✗ Invalid fact (id_property_ == 0)";
                delete fact;
            }
            LOG_INFO << "========================================";
        } catch (const std::exception& e) {
            LOG_ERROR << "Error processing fact: " << e.what();
        }
    }

    // Convert Graph to ROS message
    procedural_interfaces::StateMachineGraph convertGraphToMsg(Graph* graph) {
        procedural_interfaces::StateMachineGraph msg;

        if (!graph) return msg;

        msg.graph_id = std::to_string(reinterpret_cast<uintptr_t>(graph));
        msg.graph_name = graph->getName();
        msg.action_type = "action";  // Could be extracted from graph if available
        msg.state = GraphStateToString(graph->getState());
        msg.confidence_score = graph->getCompletionRatio();
        msg.start_time = ros::Time::now();  // Could track actual start time
        msg.last_update = ros::Time::now();

        // Add current node ID
        auto current_node = graph->getCurrentNode();
        if (current_node) {
            msg.current_node_id = std::to_string(current_node->getId());
        }

        // Add instantiated variables
        auto& table_vars = graph->getTableVariables();
        for (const auto& var_pair : table_vars.variables) {
            msg.instantiated_variables.push_back(convertVariableToMsg(var_pair.first, var_pair.second));
        }

        // Get initial and final nodes for node type detection
        auto initial_node = graph->getInitialNode();
        auto final_node = graph->getFinalNode();
        uint64_t current_node_id = current_node ? current_node->getId() : 0;

        // Add all nodes
        for (const auto& node_pair : graph->getNodes()) {
            auto node = node_pair.second;
            procedural_interfaces::StateMachineNode node_msg;
            node_msg.node_id = std::to_string(node->getId());
            node_msg.node_name = node->getFullName();
            node_msg.is_active = (node->getId() == current_node_id);
            node_msg.activation_level = node_msg.is_active ? 1.0 : 0.0;

            // Determine node type
            if (initial_node && node->getId() == initial_node->getId()) {
                node_msg.node_type = "initial";
            } else if (final_node && node->getId() == final_node->getId()) {
                node_msg.node_type = "final";
            } else {
                node_msg.node_type = "intermediate";
            }

            msg.nodes.push_back(node_msg);

            // Add transitions from this node
            for (const auto& transition : node->getTransitions()) {
                procedural_interfaces::StateMachineTransition trans_msg;
                trans_msg.from_node_id = std::to_string(transition->getSourceId());
                trans_msg.to_node_id = std::to_string(transition->getTargetId());
                trans_msg.condition_type = "observation";
                trans_msg.condition_value = transition->getObservation()->toString();

                // Check if this transition has been satisfied (activated)
                // A transition is satisfied if its source node is before or equal to current node
                // and its target node has been reached
                uint64_t source_id = transition->getSourceId();
                uint64_t target_id = transition->getTargetId();

                // Simple heuristic: transition is satisfied if we've passed its source node
                // and the target is <= current (assuming linear progression)
                trans_msg.is_satisfied = (target_id <= current_node_id && source_id < current_node_id);

                // Extract flags from observation if it's an ObservationFact
                auto obs_fact = dynamic_cast<ObservationFact*>(transition->getObservation());
                if (obs_fact) {
                    trans_msg.is_negative = !obs_fact->getFact().getAdd();
                    trans_msg.is_required = obs_fact->getFact().isRequired();
                } else {
                    trans_msg.is_negative = false;
                    trans_msg.is_required = false;
                }

                msg.transitions.push_back(trans_msg);
            }
        }

        // TODO: last_used_transition_id would require tracking in Graph class

        return msg;
    }

    // Publish initial library of all built graphs
    void publishInitialLibrary() {
        procedural_interfaces::StateMachineLibrary lib_msg;

        if (action_builder_) {
            auto actions = action_builder_->getActions();
            lib_msg.total_count = actions.size();
            lib_msg.active_count = 0;  // No active graphs initially
            lib_msg.completed_count = 0;  // No completed graphs initially

            // Convert each action to a StateMachineGraph message
            for (const auto& action : actions) {
                procedural_interfaces::StateMachineGraph graph_msg;
                graph_msg.graph_name = action->getName();
                graph_msg.action_type = "action";
                graph_msg.confidence_score = 1.0;
                graph_msg.start_time = ros::Time::now();
                graph_msg.last_update = ros::Time::now();

                // Get the factory graph from the action
                auto factory = action->getFactory();
                if (factory) {
                    graph_msg.graph_id = std::to_string(factory->getId());
                    graph_msg.state = GraphStateToString(factory->getState());

                    auto initial_node = factory->getInitialNode();
                    auto final_node = factory->getFinalNode();

                    // Convert nodes
                    for (const auto& node_pair : factory->getNodes()) {
                        auto node = node_pair.second;
                        procedural_interfaces::StateMachineNode node_msg;
                        node_msg.node_id = std::to_string(node->getId());
                        node_msg.node_name = node->getFullName();
                        node_msg.is_active = false;
                        node_msg.activation_level = 0.0;

                        // Determine node type
                        if (initial_node && node->getId() == initial_node->getId()) {
                            node_msg.node_type = "initial";
                        } else if (final_node && node->getId() == final_node->getId()) {
                            node_msg.node_type = "final";
                        } else {
                            node_msg.node_type = "intermediate";
                        }

                        graph_msg.nodes.push_back(node_msg);

                        // Convert transitions from this node
                        for (const auto& transition : node->getTransitions()) {
                            procedural_interfaces::StateMachineTransition trans_msg;
                            trans_msg.from_node_id = std::to_string(transition->getSourceId());
                            trans_msg.to_node_id = std::to_string(transition->getTargetId());
                            trans_msg.condition_type = "observation";
                            trans_msg.condition_value = transition->getObservation()->toString();
                            trans_msg.is_satisfied = false;

                            // Extract flags from observation if it's an ObservationFact
                            auto obs_fact = dynamic_cast<ObservationFact*>(transition->getObservation());
                            if (obs_fact) {
                                trans_msg.is_negative = !obs_fact->getFact().getAdd();
                                trans_msg.is_required = obs_fact->getFact().isRequired();
                            } else {
                                trans_msg.is_negative = false;
                                trans_msg.is_required = false;
                            }

                            graph_msg.transitions.push_back(trans_msg);
                        }
                    }
                }

                lib_msg.graphs.push_back(graph_msg);
            }
        }

        graph_library_pub_.publish(lib_msg);
        LOG_INFO << "Published initial library with " << lib_msg.total_count << " state machines";
    }

    // Replace variables in description strings
    std::string replaceVariables(const std::string& str, const VariableTable_t& var_table, Graph* graph) {
        std::string result = str;

        // Replace ?? with graph ID
        size_t pos = result.find("??");
        if (pos != std::string::npos) {
            std::string graph_id = graph->getName() + "_" + std::to_string(graph->getId());
            result.replace(pos, 2, graph_id);
        }

        // Replace variables like ?L, ?executor, etc.
        for (const auto& var_pair : var_table.variables) {
            if (var_pair.second && var_pair.second->isSet()) {
                // Handle both "T" and "?T" as keys in var_table
                std::string var_placeholder = var_pair.first;
                if (var_placeholder.empty() || var_placeholder[0] != '?') {
                    var_placeholder = "?" + var_placeholder;
                }

                std::string var_value = WordTable::individuals_table.get(var_pair.second->getValue());

                LOG_DEBUG << "Replacing '" << var_placeholder << "' with '" << var_value << "' in: " << result;

                pos = 0;
                while ((pos = result.find(var_placeholder, pos)) != std::string::npos) {
                    result.replace(pos, var_placeholder.length(), var_value);
                    pos += var_value.length();
                }
            }
        }

        return result;
    }

    // Service callbacks
    bool getCurrentStateCallback(procedural_interfaces::GetCurrentState::Request& req,
                                 procedural_interfaces::GetCurrentState::Response& res) {
        // Implementation would query ActionRecognition for current state
        res.total_active = 0;
        res.total_hypothesis = 0;
        res.total_completed = 0;
        return true;
    }

    bool getGraphDetailsCallback(procedural_interfaces::GetGraphDetails::Request& req,
                                 procedural_interfaces::GetGraphDetails::Response& res) {
        // Implementation would find specific graph by ID
        res.success = false;
        res.error_message = "Graph not found";
        return true;
    }

    bool resetRecognitionCallback(procedural_interfaces::ResetRecognition::Request& req,
                                  procedural_interfaces::ResetRecognition::Response& res) {
        // Implementation would reset recognition state
        res.success = true;
        res.graphs_reset = 0;
        return true;
    }

private:
    ros::NodeHandle nh_;
    ActionRecognition* action_recognition_;
    ActionBuilder* action_builder_;

    // ROS interface
    ros::Subscriber fact_sub_;
    ros::Publisher state_change_pub_;
    ros::Publisher recognized_action_pub_;
    ros::Publisher description_pub_;
    ros::Publisher description_str_pub_;
    ros::Publisher graph_library_pub_;
    ros::Publisher active_graphs_state_pub_;
    ros::ServiceServer get_current_state_srv_;
    ros::ServiceServer get_graph_details_srv_;
    ros::ServiceServer reset_recognition_srv_;

    // Parameters
    std::string memory_service_namespace_;
    std::string action_filter_;
    std::string fact_topic_;
    double fact_ttl_;
    int buffer_max_size_;
    bool debug_mode_;

    // State
    std::string last_fact_;
};

void signalHandler(int sig) {
    LOG_INFO << "Shutting down action recognition node...";
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_recognition_node");

    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        ActionRecognitionNode node;
        node.spin();
    } catch (const std::exception& e) {
        LOG_ERROR << "Action recognition node failed: " << e.what();
        return 1;
    }

    LOG_INFO << "Action recognition node shutdown complete";
    return 0;
}