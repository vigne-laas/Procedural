#ifndef PROCEDURAL_ACTION_RECOGNITION_NODE_H
#define PROCEDURAL_ACTION_RECOGNITION_NODE_H

#include <ros/ros.h>
#include <functional>
#include <memory>

#include "ontologenius/OntologyManipulator.h"
#include "mementar/TimelineManipulator.h"
#include "mementar/StampedFact.h"

#include "procedural/action_recognition/reader/MemoryActionReader.h"
#include "procedural/action_recognition/builder/ActionBuilder.h"
#include "procedural/action_recognition/core/ActionRecognition.h"

// Messages
#include "procedural_interfaces/RecognitionStateChange.h"
#include "procedural_interfaces/RecognizedAction.h"
#include "procedural_interfaces/StateMachineGraph.h"

namespace procedural {

class ActionRecognitionNode {
public:
    // Constructor for multi-agent architecture with dedicated ontology and timeline
    ActionRecognitionNode(ros::NodeHandle* nh,
                          onto::OntologyManipulator* onto_manipulator,
                          mementar::TimelineManipulator* timeline_manipulator,
                          const std::string& agent_name);

    ~ActionRecognitionNode();

    // Initialize action recognition system
    bool initializeActionRecognition();

    // Main processing loop
    void spin();

    // Stop the node
    void stop() { run_ = false; }
    inline bool isRunning() const { return run_; }

    // Callbacks to MultiNodeManager (centralized publication)
    void setRecognizedActionCallback(std::function<void(const procedural_interfaces::RecognizedAction&)> cb) {
        recognized_action_callback_ = cb;
    }

    void setStateChangeCallback(std::function<void(const procedural_interfaces::RecognitionStateChange&)> cb) {
        state_change_callback_ = cb;
    }

    void setActiveGraphsCallback(std::function<void(const std::vector<Graph*>&, const std::string&)> cb) {
        active_graphs_callback_ = cb;
    }

    void setDescriptionCallback(std::function<void(const procedural_interfaces::Description&)> cb) {
        description_callback_ = cb;
    }

    // Utility methods (public for MultiNodeManager access)
    procedural_interfaces::StateMachineGraph convertGraphToMsg(Graph* graph);
    procedural_interfaces::Variable convertVariableToMsg(const std::string& name, const std::shared_ptr<Variable_t>& var);
    std::string replaceVariables(const std::string& str, const VariableTable_t& var_table, Graph* graph);

    // Access to action builder for library publication
    ActionBuilder* getActionBuilder() const { return action_builder_; }

private:
    // Event handlers
    void handleRecognizedGraphs(const std::vector<Graph*>& graphs);
    void handleActiveGraphsUpdate(const std::vector<Graph*>& graphs);

    // ROS callback
    void factCallback(const mementar::StampedFact::ConstPtr& msg);

    // ROS interface
    ros::NodeHandle* nh_;
    ros::Subscriber fact_sub_;

    // Agent identification
    std::string agent_name_;  // robot_01, client_05, etc.

    // Ontology and timeline manipulators (dedicated per agent)
    onto::OntologyManipulator* onto_manipulator_;
    mementar::TimelineManipulator* timeline_manipulator_;

    // Action recognition components
    ActionRecognition* action_recognition_;
    ActionBuilder* action_builder_;

    // Callbacks to MultiNodeManager (centralized publication)
    std::function<void(const procedural_interfaces::RecognizedAction&)> recognized_action_callback_;
    std::function<void(const procedural_interfaces::RecognitionStateChange&)> state_change_callback_;
    std::function<void(const std::vector<Graph*>&, const std::string&)> active_graphs_callback_;
    std::function<void(const procedural_interfaces::Description&)> description_callback_;

    // Parameters
    std::string memory_service_namespace_;
    std::string action_filter_;
    double fact_ttl_;
    int buffer_max_size_;
    bool debug_mode_;
    bool run_;

    // State
    std::string last_fact_;
};

} // namespace procedural

#endif // PROCEDURAL_ACTION_RECOGNITION_NODE_H
