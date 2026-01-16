/**
 * @file commitment_test_monitor.cpp
 * @brief Test monitor that verifies commitment system behavior
 *
 * This node monitors commitment events and validates that the system
 * behaves correctly according to the expected test scenario.
 */

#include <ros/ros.h>
#include <procedural_interfaces/CommitmentEvent.h>
#include <yggdrasil_interfaces/Event.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <map>

class CommitmentTestMonitor {
public:
    CommitmentTestMonitor(ros::NodeHandle& nh)
        : nh_(nh), test_passed_(false), test_started_(false) {

        // Get parameters
        nh_.param<std::string>("expected_scenario", expected_scenario_, "violation");
        nh_.param<double>("timeout_sec", timeout_, 60.0);

        // Subscribe to commitment events
        commitment_events_sub_ = nh_.subscribe("/commitment/events", 100,
            &CommitmentTestMonitor::commitmentEventCallback, this);

        // Subscribe to Yggdrasil events
        yggdrasil_events_sub_ = nh_.subscribe("/yggdrasil/events", 100,
            &CommitmentTestMonitor::yggdrasilEventCallback, this);

        ROS_INFO("========================================");
        ROS_INFO("    Commitment Test Monitor");
        ROS_INFO("========================================");
        ROS_INFO("Expected Scenario: %s", expected_scenario_.c_str());
        ROS_INFO("Timeout: %.1f seconds", timeout_);
        ROS_INFO("Monitoring commitment system behavior...\n");

        // Start timeout timer
        timeout_timer_ = nh_.createTimer(ros::Duration(timeout_),
            &CommitmentTestMonitor::timeoutCallback, this, true);
    }

    void printSummary() {
        ROS_INFO("\n========================================");
        ROS_INFO("    TEST SUMMARY");
        ROS_INFO("========================================");
        ROS_INFO("Expected Scenario: %s", expected_scenario_.c_str());
        ROS_INFO("Events Received:");
        ROS_INFO("  - MADE: %zu", made_events_.size());
        ROS_INFO("  - ACTIVATED: %zu", activated_events_.size());
        ROS_INFO("  - FULFILLED: %zu", fulfilled_events_.size());
        ROS_INFO("  - CONDITION_VIOLATED: %zu", violated_events_.size());
        ROS_INFO("  - DROPPED: %zu", dropped_events_.size());
        ROS_INFO("Yggdrasil Events: %zu", yggdrasil_events_.size());

        if (test_passed_) {
            ROS_INFO("\n✅ TEST PASSED");
        } else {
            ROS_WARN("\n❌ TEST FAILED");
            printFailureReasons();
        }
        ROS_INFO("========================================\n");
    }

    bool testPassed() const {
        return test_passed_;
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber commitment_events_sub_;
    ros::Subscriber yggdrasil_events_sub_;
    ros::Timer timeout_timer_;

    std::string expected_scenario_;
    double timeout_;
    bool test_passed_;
    bool test_started_;

    // Event tracking
    std::vector<procedural_interfaces::CommitmentEvent> made_events_;
    std::vector<procedural_interfaces::CommitmentEvent> activated_events_;
    std::vector<procedural_interfaces::CommitmentEvent> fulfilled_events_;
    std::vector<procedural_interfaces::CommitmentEvent> violated_events_;
    std::vector<procedural_interfaces::CommitmentEvent> dropped_events_;
    std::vector<yggdrasil_interfaces::Event> yggdrasil_events_;

    std::vector<std::string> failure_reasons_;

    void commitmentEventCallback(const procedural_interfaces::CommitmentEvent::ConstPtr& msg) {
        test_started_ = true;

        ROS_INFO("📨 Commitment Event: %s (commitment_id: %s, action: %s)",
                 msg->event_type.c_str(), msg->commitment_id.c_str(), msg->action_name.c_str());

        if (msg->event_type == "MADE") {
            made_events_.push_back(*msg);
        } else if (msg->event_type == "ACTIVATED") {
            activated_events_.push_back(*msg);
        } else if (msg->event_type == "FULFILLED") {
            fulfilled_events_.push_back(*msg);
        } else if (msg->event_type == "CONDITION_VIOLATED") {
            violated_events_.push_back(*msg);
            ROS_INFO("   Condition Type: %s", msg->condition_type.c_str());
            ROS_INFO("   Recovery Action: %s", msg->reaction_action.c_str());
        } else if (msg->event_type == "DROPPED") {
            dropped_events_.push_back(*msg);
        }

        // Check if test is complete
        checkTestCompletion();
    }

    void yggdrasilEventCallback(const yggdrasil_interfaces::Event::ConstPtr& msg) {
        if (msg->is_deactivation) {
            ROS_INFO("🔔 Yggdrasil Deactivation Event (registration_id: %ld)", msg->registration_id);
            yggdrasil_events_.push_back(*msg);
        }
    }

    void checkTestCompletion() {
        if (expected_scenario_ == "success") {
            checkSuccessScenario();
        } else if (expected_scenario_ == "violation") {
            checkViolationScenario();
        } else if (expected_scenario_ == "multi_violation") {
            checkMultiViolationScenario();
        }
    }

    void checkSuccessScenario() {
        // Expected: MADE → ACTIVATED → FULFILLED
        // No violations
        if (made_events_.size() >= 1 &&
            activated_events_.size() >= 1 &&
            fulfilled_events_.size() >= 1 &&
            violated_events_.empty()) {

            ROS_INFO("\n✓ Success scenario validated:");
            ROS_INFO("  - Commitment created (MADE)");
            ROS_INFO("  - Commitment activated (ACTIVATED)");
            ROS_INFO("  - Commitment fulfilled (FULFILLED)");
            ROS_INFO("  - No violations detected");

            test_passed_ = true;
            printSummary();
            ros::shutdown();
        }
    }

    void checkViolationScenario() {
        // Expected: MADE → ACTIVATED → CONDITION_VIOLATED
        // At least one Yggdrasil deactivation event
        if (made_events_.size() >= 1 &&
            activated_events_.size() >= 1 &&
            violated_events_.size() >= 1 &&
            yggdrasil_events_.size() >= 1) {

            // Verify violation contains recovery action
            bool has_recovery_action = false;
            for (const auto& event : violated_events_) {
                if (!event.reaction_action.empty()) {
                    has_recovery_action = true;
                    break;
                }
            }

            if (has_recovery_action) {
                ROS_INFO("\n✓ Violation scenario validated:");
                ROS_INFO("  - Commitment created (MADE)");
                ROS_INFO("  - Commitment activated (ACTIVATED)");
                ROS_INFO("  - Yggdrasil detected condition violation");
                ROS_INFO("  - CommitmentMonitor published VIOLATED event");
                ROS_INFO("  - Recovery action specified");

                test_passed_ = true;
                printSummary();
                ros::shutdown();
            } else {
                failure_reasons_.push_back("Violation event missing recovery action");
            }
        }
    }

    void checkMultiViolationScenario() {
        // Expected: Multiple violations with different condition types
        if (made_events_.size() >= 1 &&
            activated_events_.size() >= 1 &&
            violated_events_.size() >= 2) {

            // Count different condition types
            std::map<std::string, int> condition_counts;
            for (const auto& event : violated_events_) {
                condition_counts[event.condition_type]++;
            }

            if (condition_counts.size() >= 2) {
                ROS_INFO("\n✓ Multi-violation scenario validated:");
                ROS_INFO("  - Multiple violations detected: %zu", violated_events_.size());
                ROS_INFO("  - Different condition types: %zu", condition_counts.size());
                for (const auto& pair : condition_counts) {
                    ROS_INFO("    - %s: %d violations", pair.first.c_str(), pair.second);
                }

                test_passed_ = true;
                printSummary();
                ros::shutdown();
            } else {
                failure_reasons_.push_back("Not enough different condition types violated");
            }
        }
    }

    void timeoutCallback(const ros::TimerEvent& event) {
        if (!test_passed_) {
            ROS_WARN("\n⏱️  TEST TIMEOUT REACHED");
            if (!test_started_) {
                failure_reasons_.push_back("No events received - system may not be running");
            } else {
                failure_reasons_.push_back("Test did not complete within timeout");
            }
            printSummary();
        }
        ros::shutdown();
    }

    void printFailureReasons() {
        if (!failure_reasons_.empty()) {
            ROS_WARN("Failure Reasons:");
            for (const auto& reason : failure_reasons_) {
                ROS_WARN("  - %s", reason.c_str());
            }
        }

        // Print what's missing
        if (expected_scenario_ == "success") {
            if (made_events_.empty()) ROS_WARN("  - Missing MADE event");
            if (activated_events_.empty()) ROS_WARN("  - Missing ACTIVATED event");
            if (fulfilled_events_.empty()) ROS_WARN("  - Missing FULFILLED event");
            if (!violated_events_.empty()) ROS_WARN("  - Unexpected VIOLATED events");
        } else if (expected_scenario_ == "violation") {
            if (made_events_.empty()) ROS_WARN("  - Missing MADE event");
            if (activated_events_.empty()) ROS_WARN("  - Missing ACTIVATED event");
            if (violated_events_.empty()) ROS_WARN("  - Missing VIOLATED event");
            if (yggdrasil_events_.empty()) ROS_WARN("  - Missing Yggdrasil deactivation event");
        } else if (expected_scenario_ == "multi_violation") {
            if (violated_events_.size() < 2) {
                ROS_WARN("  - Not enough violations (expected >= 2, got %zu)", violated_events_.size());
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "commitment_test_monitor");
    ros::NodeHandle nh("~");

    CommitmentTestMonitor monitor(nh);

    ros::spin();

    // Exit with appropriate code
    return monitor.testPassed() ? 0 : 1;
}
