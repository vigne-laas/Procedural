#include "procedural/ActionRecognitionNode.h"

#include <std_msgs/String.h>
#include "procedural/utils/Logger.h"
#include "procedural/utils/TimeStamp.h"
#include "procedural/structures/Fact.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/structures/graph/Graph.h"  // Pour GraphStateToString()

// Messages
#include "procedural_interfaces/StateMachineNode.h"
#include "procedural_interfaces/StateMachineTransition.h"
#include "procedural_interfaces/Description.h"
#include "procedural_interfaces/Triplet.h"
#include "procedural_interfaces/Argument.h"

using namespace procedural;

// Helper function to trim whitespace
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, last - first + 1);
}

ActionRecognitionNode::ActionRecognitionNode(ros::NodeHandle* nh,
                                             onto::OntologyManipulator* onto_manipulator,
                                             mementar::TimelineManipulator* timeline_manipulator,
                                             const std::string& agent_name)
    : nh_(nh)
    , agent_name_(agent_name)
    , onto_manipulator_(onto_manipulator)
    , timeline_manipulator_(timeline_manipulator)
    , action_recognition_(nullptr)
    , action_builder_(nullptr)
    , run_(true)
{
    // Initialize parameters with agent-specific namespace
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("memory_service_namespace", memory_service_namespace_, "procedural_memory");
    private_nh.param<std::string>("action_filter", action_filter_, "");
    private_nh.param<double>("fact_time_to_live", fact_ttl_, 10.0);
    private_nh.param<int>("buffer_max_size", buffer_max_size_, 1000);
    private_nh.param<bool>("debug_mode", debug_mode_, false);

    LOG_INFO << "[" << agent_name_ << "] Action Recognition Node starting with parameters:";
    LOG_INFO << "[" << agent_name_ << "]   Memory service namespace: " << memory_service_namespace_;
    LOG_INFO << "[" << agent_name_ << "]   Action filter: " << (action_filter_.empty() ? "none" : action_filter_);
    LOG_INFO << "[" << agent_name_ << "]   Fact TTL: " << fact_ttl_ << " seconds";
    LOG_INFO << "[" << agent_name_ << "]   Buffer max size: " << buffer_max_size_;
    LOG_INFO << "[" << agent_name_ << "]   Debug mode: " << (debug_mode_ ? "enabled" : "disabled");

    // Initialize ontology and timeline
    if (onto_manipulator_) {
        onto_manipulator_->close();
        LOG_INFO << "[" << agent_name_ << "] Ontology manipulator initialized";
    }

    if (timeline_manipulator_) {
        LOG_INFO << "[" << agent_name_ << "] Timeline manipulator initialized";
    }
}

ActionRecognitionNode::~ActionRecognitionNode() {
    if (action_recognition_) {
        delete action_recognition_;
    }
    if (action_builder_) {
        delete action_builder_;
    }
}

bool ActionRecognitionNode::initializeActionRecognition() {
    try {
        // Create memory reader
        auto memory_reader = std::make_unique<MemoryActionReader>(*nh_, memory_service_namespace_);

        // Wait for memory service
        LOG_INFO << "[" << agent_name_ << "] Waiting for memory service to become available...";
        if (!memory_reader->waitForService(ros::Duration(30.0))) {
            LOG_ERROR << "[" << agent_name_ << "] Memory service not available after 30 seconds";
            return false;
        }

        // Read actions from memory
        LOG_INFO << "[" << agent_name_ << "] Reading actions from memory service...";
        if (!memory_reader->read(action_filter_)) {
            LOG_ERROR << "[" << agent_name_ << "] Failed to read actions from memory service";
            return false;
        }

        LOG_INFO << "[" << agent_name_ << "] Successfully loaded " << memory_reader->getActionCount() << " actions";

        // Build actions using ActionBuilder
        LOG_INFO << "[" << agent_name_ << "] Building action state machines...";
        auto simple_actions = memory_reader->getSimpleActions();
        auto composed_actions = memory_reader->getComposedActions();

        action_builder_ = new ActionBuilder(simple_actions, composed_actions, "/tmp/procedural_debug");
        auto actions = action_builder_->getActions();

        if (actions.empty()) {
            LOG_ERROR << "[" << agent_name_ << "] No actions were built successfully";
            return false;
        }

        LOG_INFO << "[" << agent_name_ << "] Successfully built " << actions.size() << " action state machines";

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

        LOG_INFO << "[" << agent_name_ << "] Action recognition system initialized successfully";
        LOG_INFO << "[" << agent_name_ << "] Properties in WordTable:\n" << WordTable::properties_table.toString();

        // Subscribe to dedicated timeline topic
        std::string topic = agent_name_.empty() ? "/mementar/echo" : "/mementar/echo/" + agent_name_;
        fact_sub_ = nh_->subscribe<mementar::StampedFact>(topic, buffer_max_size_,
                                                           &ActionRecognitionNode::factCallback, this);
        LOG_INFO << "[" << agent_name_ << "] Subscribed to: " << topic;

        return true;

    } catch (const std::exception& e) {
        LOG_ERROR << "[" << agent_name_ << "] Exception during initialization: " << e.what();
        return false;
    }
}

void ActionRecognitionNode::handleRecognizedGraphs(const std::vector<Graph*>& graphs) {
    for (const auto& graph : graphs) {
        LOG_INFO << "[" << agent_name_ << "] Recognized action graph: " << graph->getName();

        // Create recognized action message
        procedural_interfaces::RecognizedAction action_msg;
        action_msg.header.stamp = ros::Time::now();
        action_msg.timeline_id = agent_name_;  // ADDED: timeline identification
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
                arg.value = WordTable::individuals_table.get(var_pair.second->getValue());  // Bound value
                action_msg.arguments.push_back(arg);
                LOG_DEBUG << "[" << agent_name_ << "]   Bound variable: " << arg.literal << " (" << arg.type << ") = " << arg.value;
            }
        }

        // Call callback to MultiNodeManager (centralized publication)
        if (recognized_action_callback_) {
            recognized_action_callback_(action_msg);
        }

        // Publish descriptions if available
        LOG_DEBUG << "[" << agent_name_ << "] Checking for descriptions for graph: " << graph->getName();
        if (action_builder_) {
            auto actions = action_builder_->getActions();
            // Find the action corresponding to this graph
            for (const auto& action : actions) {
                if (action->getName() == graph->getName()) {
                    auto descriptions = action->getDescriptions();
                    if (!descriptions.descriptions.empty()) {
                        procedural_interfaces::Description desc_msg;

                        for (const auto& parsed_desc : descriptions.descriptions) {
                            procedural_interfaces::Triplet triplet;
                            triplet.add = true;  // Descriptions are assertions
                            triplet.required = false;

                            // Replace variables in subject, property, object
                            triplet.subject.literal = replaceVariables(parsed_desc.subject, var_table, graph);
                            triplet.property = replaceVariables(parsed_desc.property, var_table, graph);
                            triplet.object.literal = replaceVariables(parsed_desc.object, var_table, graph);

                            desc_msg.descriptions.push_back(triplet);
                            LOG_DEBUG << "[" << agent_name_ << "]   Description: " << triplet.subject.literal << " "
                                     << triplet.property << " " << triplet.object.literal;
                        }

                        LOG_INFO << "[" << agent_name_ << "] Publishing " << desc_msg.descriptions.size()
                                 << " descriptions for action " << graph->getName();

                        // Call callback to MultiNodeManager
                        if (description_callback_) {
                            description_callback_(desc_msg);
                        }
                    }
                    break;
                }
            }
        }

        // Create state change message
        procedural_interfaces::RecognitionStateChange state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.timeline_id = agent_name_;  // ADDED: timeline identification
        state_msg.event_type = "graph_completed";
        state_msg.affected_graph = convertGraphToMsg(graph);
        state_msg.old_state = "Active";
        state_msg.new_state = "Completed";
        state_msg.triggering_fact = last_fact_;

        // Call callback to MultiNodeManager (centralized publication)
        if (state_change_callback_) {
            state_change_callback_(state_msg);
        }
    }
}

void ActionRecognitionNode::handleActiveGraphsUpdate(const std::vector<Graph*>& graphs) {
    // Log active graphs
    for (const auto& graph : graphs) {
        LOG_INFO << "[" << agent_name_ << "] Active graph updated: " << graph->getName()
                 << " completion: " << graph->getCompletionRatio();
    }

    // Publish state change for each graph individually
    for (const auto& graph : graphs) {
        procedural_interfaces::RecognitionStateChange state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.timeline_id = agent_name_;  // ADDED: timeline identification
        state_msg.event_type = "graph_progressed";
        state_msg.affected_graph = convertGraphToMsg(graph);
        state_msg.old_state = "Active";
        state_msg.new_state = "Active";
        state_msg.triggering_fact = last_fact_;

        // Call callback to MultiNodeManager
        if (state_change_callback_) {
            state_change_callback_(state_msg);
        }
    }

    // Call active graphs callback to MultiNodeManager (for centralized aggregation)
    if (active_graphs_callback_) {
        active_graphs_callback_(graphs, agent_name_);
    }
}

void ActionRecognitionNode::factCallback(const mementar::StampedFact::ConstPtr& msg) {
    try {
        // Log raw ROS message
        LOG_INFO << "[" << agent_name_ << "] ========== RECEIVED ROS FACT ==========";
        LOG_INFO << "[" << agent_name_ << "]   added: " << (msg->added ? "true" : "false");
        LOG_INFO << "[" << agent_name_ << "]   subject: '" << msg->subject << "'";
        LOG_INFO << "[" << agent_name_ << "]   predicat: '" << msg->predicat << "'";
        LOG_INFO << "[" << agent_name_ << "]   object: '" << msg->object << "'";

        // CRITICAL FIX: Use msg->stamp for temporal precision (not ros::Time::now())
        Fact* fact = new Fact(msg->added, msg->subject, "unknown", msg->predicat,
                             msg->object, "unknown",
                             TimeStamp_t({msg->stamp.sec, msg->stamp.nsec}));

        if (fact->isValid()) {
            LOG_INFO << "[" << agent_name_ << "]   Converted fact: " << fact->toString();
            action_recognition_->addToQueue(fact);
            last_fact_ = fact->toString();
            LOG_INFO << "[" << agent_name_ << "]   ✓ Added to queue";
        } else {
            LOG_ERROR << "[" << agent_name_ << "]   ✗ Invalid fact (id_property_ == 0)";
            delete fact;
        }
        LOG_INFO << "[" << agent_name_ << "] ========================================";
    } catch (const std::exception& e) {
        LOG_ERROR << "[" << agent_name_ << "] Error processing fact: " << e.what();
    }
}

procedural_interfaces::StateMachineGraph ActionRecognitionNode::convertGraphToMsg(Graph* graph) {
    procedural_interfaces::StateMachineGraph msg;

    if (!graph) return msg;

    msg.timeline_id = agent_name_;  // ADDED: timeline identification
    msg.graph_id = std::to_string(reinterpret_cast<uintptr_t>(graph));
    msg.graph_name = graph->getName();
    msg.action_type = "action";
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

            uint64_t source_id = transition->getSourceId();
            uint64_t target_id = transition->getTargetId();
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

    return msg;
}

procedural_interfaces::Variable ActionRecognitionNode::convertVariableToMsg(const std::string& name, const std::shared_ptr<Variable_t>& var) {
    procedural_interfaces::Variable var_msg;
    var_msg.literal = name;
    var_msg.type = var->getType();
    var_msg.isVariable = !var->isSet();
    var_msg.isClass = false;
    var_msg.value = var->isSet() ? WordTable::individuals_table[var->getValue()] : "";
    return var_msg;
}

std::string ActionRecognitionNode::replaceVariables(const std::string& str, const VariableTable_t& var_table, Graph* graph) {
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

            LOG_DEBUG << "[" << agent_name_ << "] Replacing '" << var_placeholder << "' with '" << var_value << "' in: " << result;

            pos = 0;
            while ((pos = result.find(var_placeholder, pos)) != std::string::npos) {
                result.replace(pos, var_placeholder.length(), var_value);
                pos += var_value.length();
            }
        }
    }

    return result;
}

void ActionRecognitionNode::spin() {
    if (!action_recognition_) {
        LOG_ERROR << "[" << agent_name_ << "] Action recognition not initialized, cannot spin";
        return;
    }

    ros::Rate rate(10); // 10 Hz

    while (ros::ok() && run_) {
        // Process any incoming ROS messages
        ros::spinOnce();

        // Process the fact queue
        action_recognition_->processQueue(TimeStamp_t(ros::Time::now().toSec()));

        rate.sleep();
    }
}
