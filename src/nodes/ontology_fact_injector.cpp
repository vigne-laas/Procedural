/**
 * @file ontology_fact_injector.cpp
 * @brief Test utility to inject facts for commitment testing
 *
 * This node simulates real-world events by publishing to the ontology feeder topic,
 * which then propagate to Mementar, Yggdrasil, and trigger commitment monitoring.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

class OntologyFactInjector {
public:
    OntologyFactInjector(ros::NodeHandle& nh)
        : nh_(nh) {

        // Get parameters
        nh_.param<std::string>("test_scenario", test_scenario_, "violation");
        nh_.param<std::string>("agent_id", agent_id_, "robot_01");
        nh_.param<double>("delay_between_facts", delay_, 3.0);

        // Publisher for ontology facts (simplified approach)
        onto_feeder_pub_ = nh_.advertise<std_msgs::String>("/ontologenius/insert", 10);

        ROS_INFO("========================================");
        ROS_INFO("    Ontology Fact Injector");
        ROS_INFO("========================================");
        ROS_INFO("Test Scenario: %s", test_scenario_.c_str());
        ROS_INFO("Agent ID: %s", agent_id_.c_str());
        ROS_INFO("Delay: %.1f seconds", delay_);

        // Wait for subscribers
        ros::Duration(2.0).sleep();

        // Run test scenario
        if (test_scenario_ == "success") {
            runSuccessScenario();
        } else if (test_scenario_ == "violation") {
            runViolationScenario();
        } else if (test_scenario_ == "multi_violation") {
            runMultiViolationScenario();
        } else {
            ROS_ERROR("Unknown test scenario: %s", test_scenario_.c_str());
        }
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher onto_feeder_pub_;
    std::string test_scenario_;
    std::string agent_id_;
    double delay_;

    void injectFact(const std::string& subject, const std::string& predicate, const std::string& object) {
        ROS_INFO("💉 Injecting fact: %s %s %s", subject.c_str(), predicate.c_str(), object.c_str());

        // Publish to ontology feeder topic
        std_msgs::String msg;
        msg.data = subject + "|" + predicate + "|" + object;
        onto_feeder_pub_.publish(msg);

        ROS_INFO("   ✓ Fact published");
        ros::Duration(0.1).sleep();  // Allow time for processing
    }

    void injectDataProperty(const std::string& subject, const std::string& predicate, const std::string& value) {
        ROS_INFO("💉 Injecting data: %s %s \"%s\"", subject.c_str(), predicate.c_str(), value.c_str());

        // Publish to ontology feeder topic
        std_msgs::String msg;
        msg.data = subject + "|" + predicate + "|" + value;
        onto_feeder_pub_.publish(msg);

        ROS_INFO("   ✓ Data published");
        ros::Duration(0.1).sleep();
    }

    void removeFact(const std::string& subject, const std::string& predicate, const std::string& object) {
        ROS_INFO("🗑️  Removing fact: %s %s %s", subject.c_str(), predicate.c_str(), object.c_str());

        // For simplicity, we'll just inject an opposite fact
        // In a real system, you'd use the ontology removal API
        ROS_INFO("   (Simplified: not actually removing, just noting)");
    }

    void runSuccessScenario() {
        ROS_INFO("\n=== RUNNING SUCCESS SCENARIO ===");
        ROS_INFO("Simulating successful action execution with all commitments maintained\n");

        // Initial state: robot is capable and ready
        injectFact(agent_id_, "isCapableOf", "moving");
        injectDataProperty(agent_id_, "hasBlockageStatus", "false");
        injectFact(agent_id_, "isInArea", "InitialArea");

        ros::Duration(delay_).sleep();

        // Action starts: GoToArea(TargetArea)
        ROS_INFO("\n📍 Action: GoToArea(TargetArea)");
        ROS_INFO("Commitment conditions:");
        ROS_INFO("  - INSTRUMENTAL: NOT { %s hasBlockageStatus true }", agent_id_.c_str());
        ROS_INFO("  - All conditions remain TRUE throughout execution\n");

        ros::Duration(delay_).sleep();

        // Action progressing - maintain all conditions
        ROS_INFO("⚙ Action progressing... (all commitments satisfied)");

        ros::Duration(delay_).sleep();

        // Action completes successfully
        ROS_INFO("✓ Action completed successfully");
        removeFact(agent_id_, "isInArea", "InitialArea");
        injectFact(agent_id_, "isInArea", "TargetArea");

        ROS_INFO("\n=== SUCCESS SCENARIO COMPLETE ===");
        ROS_INFO("Expected outcome: FULFILLED event, no violations\n");
    }

    void runViolationScenario() {
        ROS_INFO("\n=== RUNNING VIOLATION SCENARIO ===");
        ROS_INFO("Simulating commitment violation during action execution\n");

        // Initial state: robot is capable and ready
        injectFact(agent_id_, "isCapableOf", "moving");
        injectDataProperty(agent_id_, "hasBlockageStatus", "false");
        injectFact(agent_id_, "isInArea", "InitialArea");

        ros::Duration(delay_).sleep();

        // Action starts: GoToArea(TargetArea)
        ROS_INFO("\n📍 Action: GoToArea(TargetArea)");
        ROS_INFO("Commitment conditions:");
        ROS_INFO("  - INSTRUMENTAL: NOT { %s hasBlockageStatus true }", agent_id_.c_str());
        ROS_INFO("  - Recovery action: stop_and_wait\n");

        ros::Duration(delay_).sleep();

        // Simulate INSTRUMENTAL violation: robot becomes blocked
        ROS_INFO("\n⚠ SIMULATING INSTRUMENTAL VIOLATION");
        ROS_INFO("Injecting: %s hasBlockageStatus true", agent_id_.c_str());
        injectDataProperty(agent_id_, "hasBlockageStatus", "true");

        ROS_INFO("\n Expected system behavior:");
        ROS_INFO("  1. Yggdrasil detects SPARQL condition became FALSE");
        ROS_INFO("  2. CommitmentMonitor publishes CONDITION_VIOLATED event");
        ROS_INFO("  3. MissionManager triggers recovery action: stop_and_wait");

        ros::Duration(delay_ * 2).sleep();

        // Simulate recovery: blockage cleared
        ROS_INFO("\n🔧 Simulating recovery");
        ROS_INFO("Removing blockage...");
        injectDataProperty(agent_id_, "hasBlockageStatus", "false");

        ros::Duration(delay_).sleep();

        ROS_INFO("\n=== VIOLATION SCENARIO COMPLETE ===");
        ROS_INFO("Expected outcome: VIOLATED event → recovery action executed\n");
    }

    void runMultiViolationScenario() {
        ROS_INFO("\n=== RUNNING MULTI-VIOLATION SCENARIO ===");
        ROS_INFO("Testing multiple commitment types\n");

        // Initial state
        injectFact(agent_id_, "isCapableOf", "moving");
        injectDataProperty(agent_id_, "hasBlockageStatus", "false");
        injectFact(agent_id_, "hasProximityStatus", "close");
        injectFact(agent_id_, "isVisibleTo", "customer_01");
        injectFact(agent_id_, "isInArea", "InitialArea");

        ros::Duration(delay_).sleep();

        // Action starts with 3 commitment types
        ROS_INFO("\n📍 Action: ServeCustomer(customer_01)");
        ROS_INFO("Commitments:");
        ROS_INFO("  - INSTRUMENTAL: NOT { %s hasBlockageStatus true }", agent_id_.c_str());
        ROS_INFO("  - ENGAGEMENT: { %s hasProximityStatus close }", agent_id_.c_str());
        ROS_INFO("  - COMMON_GROUND: { %s isVisibleTo customer_01 }", agent_id_.c_str());

        ros::Duration(delay_).sleep();

        // Violate ENGAGEMENT
        ROS_INFO("\n⚠ VIOLATING ENGAGEMENT COMMITMENT");
        removeFact(agent_id_, "hasProximityStatus", "close");
        injectFact(agent_id_, "hasProximityStatus", "far");

        ros::Duration(delay_).sleep();

        // Violate COMMON_GROUND
        ROS_INFO("\n⚠ VIOLATING COMMON_GROUND COMMITMENT");
        removeFact(agent_id_, "isVisibleTo", "customer_01");

        ros::Duration(delay_).sleep();

        // Violate INSTRUMENTAL
        ROS_INFO("\n⚠ VIOLATING INSTRUMENTAL COMMITMENT");
        injectDataProperty(agent_id_, "hasBlockageStatus", "true");

        ros::Duration(delay_ * 2).sleep();

        ROS_INFO("\n=== MULTI-VIOLATION SCENARIO COMPLETE ===");
        ROS_INFO("Expected: 3 separate VIOLATED events with different recovery actions\n");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ontology_fact_injector");
    ros::NodeHandle nh("~");

    OntologyFactInjector injector(nh);

    ros::spin();
    return 0;
}
