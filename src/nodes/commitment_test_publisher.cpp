/**
 * @file commitment_test_publisher.cpp
 * @brief Test node that publishes simulated commitment events to test Yggdrasil integration
 *
 * This node simulates a commitment monitoring system by publishing CommitmentEvent
 * messages at different stages of the commitment lifecycle. This allows testing
 * the integration between Procedural package and Yggdrasil's ActionRecognitionDataSource.
 */

#include <ros/ros.h>
#include <procedural_interfaces/CommitmentEvent.h>
#include <procedural_interfaces/CommitmentCondition.h>
#include <string>

class CommitmentTestPublisher
{
public:
    CommitmentTestPublisher()
    {
        ros::NodeHandle nh;

        // Publisher for commitment events
        event_pub_ = nh.advertise<procedural_interfaces::CommitmentEvent>("/commitment/events", 10);

        ROS_INFO("CommitmentTestPublisher: Started");
        ROS_INFO("  Publishing to: /commitment/events");
        ROS_INFO("  Simulating commitment lifecycle for testing");
    }

    void publishTestScenario()
    {
        ROS_INFO("\n========== Starting Commitment Test Scenario ==========\n");

        // Wait for subscribers
        ros::Duration(2.0).sleep();

        // Scenario 1: Agent makes commitment to GoToArea action
        ROS_INFO("Step 1: Agent robot_01 makes commitment to GoToArea");
        publishCommitmentMade("commit_001", "robot_01", "GoToArea");
        ros::Duration(3.0).sleep();

        // Scenario 2: Commitment becomes active (all preconditions met)
        ROS_INFO("Step 2: Commitment activated (preconditions satisfied)");
        publishCommitmentActivated("commit_001", "robot_01", "GoToArea");
        ros::Duration(3.0).sleep();

        // Scenario 3: Agent makes another commitment
        ROS_INFO("Step 3: Agent robot_01 makes commitment to PickObject");
        publishCommitmentMade("commit_002", "robot_01", "PickObject");
        ros::Duration(3.0).sleep();

        // Scenario 4: First commitment is fulfilled
        ROS_INFO("Step 4: Commitment commit_001 fulfilled successfully");
        publishCommitmentFulfilled("commit_001", "robot_01", "GoToArea");
        ros::Duration(3.0).sleep();

        // Scenario 5: Second commitment is violated
        ROS_INFO("Step 5: Commitment commit_002 violated (INSTRUMENTAL failure)");
        publishCommitmentViolated("commit_002", "robot_01", "PickObject", "INSTRUMENTAL");
        ros::Duration(3.0).sleep();

        // Scenario 6: New commitment for another agent
        ROS_INFO("Step 6: Agent robot_02 makes commitment to ServeCustomer");
        publishCommitmentMade("commit_003", "robot_02", "ServeCustomer");
        ros::Duration(3.0).sleep();

        // Scenario 7: Drop the third commitment
        ROS_INFO("Step 7: Commitment commit_003 dropped (agent unavailable)");
        publishCommitmentDropped("commit_003", "robot_02", "ServeCustomer");

        ROS_INFO("\n========== Commitment Test Scenario Complete ==========\n");
        ROS_INFO("Summary:");
        ROS_INFO("  - robot_01 GoToArea: MADE -> ACTIVATED -> FULFILLED");
        ROS_INFO("  - robot_01 PickObject: MADE -> VIOLATED (INSTRUMENTAL)");
        ROS_INFO("  - robot_02 ServeCustomer: MADE -> DROPPED");
        ROS_INFO("\nYggdrasil should now have this data available via SPARQL queries.");
        ROS_INFO("Example queries:");
        ROS_INFO("  SELECT ?agent WHERE { ?agent action:hasCommitment \"GoToArea\". }");
        ROS_INFO("  SELECT ?action WHERE { robot_01 action:hasViolatedCommitment ?action. }");
        ROS_INFO("  SELECT ?agent WHERE { ?agent action:hasFulfilledCommitment \"GoToArea\". }");
    }

private:
    ros::Publisher event_pub_;

    void publishCommitmentMade(const std::string& commitment_id,
                               const std::string& agent_id,
                               const std::string& action_name)
    {
        procedural_interfaces::CommitmentEvent event;
        event.commitment_id = commitment_id;
        event.agent_id = agent_id;
        event.action_name = action_name;
        event.event_type = "MADE";
        event.condition_type = "NONE";
        event.timestamp = ros::Time::now();
        event.reaction_action = "";

        event_pub_.publish(event);
        ROS_INFO("  Published: %s made by %s for %s", commitment_id.c_str(), agent_id.c_str(), action_name.c_str());
    }

    void publishCommitmentActivated(const std::string& commitment_id,
                                     const std::string& agent_id,
                                     const std::string& action_name)
    {
        procedural_interfaces::CommitmentEvent event;
        event.commitment_id = commitment_id;
        event.agent_id = agent_id;
        event.action_name = action_name;
        event.event_type = "ACTIVATED";
        event.condition_type = "NONE";
        event.timestamp = ros::Time::now();
        event.reaction_action = "";

        event_pub_.publish(event);
        ROS_INFO("  Published: %s activated", commitment_id.c_str());
    }

    void publishCommitmentFulfilled(const std::string& commitment_id,
                                     const std::string& agent_id,
                                     const std::string& action_name)
    {
        procedural_interfaces::CommitmentEvent event;
        event.commitment_id = commitment_id;
        event.agent_id = agent_id;
        event.action_name = action_name;
        event.event_type = "FULFILLED";
        event.condition_type = "NONE";
        event.timestamp = ros::Time::now();
        event.reaction_action = "";

        event_pub_.publish(event);
        ROS_INFO("  Published: %s fulfilled", commitment_id.c_str());
    }

    void publishCommitmentViolated(const std::string& commitment_id,
                                    const std::string& agent_id,
                                    const std::string& action_name,
                                    const std::string& condition_type)
    {
        procedural_interfaces::CommitmentEvent event;
        event.commitment_id = commitment_id;
        event.agent_id = agent_id;
        event.action_name = action_name;
        event.event_type = "CONDITION_VIOLATED";
        event.condition_type = condition_type;
        event.timestamp = ros::Time::now();
        event.reaction_action = "stop_and_wait";

        // Add a violated condition for realism
        procedural_interfaces::CommitmentCondition condition;
        condition.condition_type = condition_type;
        condition.sparql_query = "SELECT * WHERE { robot hasCapability manipulate. }";
        condition.description = "Robot must have manipulation capability";
        condition.is_satisfied = false;
        event.violated_conditions.push_back(condition);

        event_pub_.publish(event);
        ROS_INFO("  Published: %s violated (%s condition failed, reaction: %s)",
                 commitment_id.c_str(), condition_type.c_str(), event.reaction_action.c_str());
    }

    void publishCommitmentDropped(const std::string& commitment_id,
                                   const std::string& agent_id,
                                   const std::string& action_name)
    {
        procedural_interfaces::CommitmentEvent event;
        event.commitment_id = commitment_id;
        event.agent_id = agent_id;
        event.action_name = action_name;
        event.event_type = "DROPPED";
        event.condition_type = "NONE";
        event.timestamp = ros::Time::now();
        event.reaction_action = "";

        event_pub_.publish(event);
        ROS_INFO("  Published: %s dropped", commitment_id.c_str());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "commitment_test_publisher");

    CommitmentTestPublisher publisher;

    // Run test scenario
    publisher.publishTestScenario();

    // Keep node alive for a bit to ensure messages are sent
    ros::Duration(2.0).sleep();

    ROS_INFO("CommitmentTestPublisher: Shutting down");
    return 0;
}
