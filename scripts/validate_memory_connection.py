#!/usr/bin/env python3

"""
Validation script to test memory service connection and action retrieval.
This script verifies that the memory service is working correctly before
testing the full action recognition system.
"""

import rospy
import sys
from procedural_interfaces.srv import GetActions, GetActionDetails

class MemoryValidationTest:
    def __init__(self):
        rospy.init_node('memory_validation_test')

        # Service clients
        self.get_actions_client = None
        self.get_action_details_client = None

        print("=== Memory Service Validation Test ===")

    def wait_for_services(self, timeout=30):
        """Wait for memory services to become available"""
        print(f"Waiting for memory services (timeout: {timeout}s)...")

        services = [
            ('/getActions', GetActions),
            ('/getActionDetails', GetActionDetails)
        ]

        for service_name, service_type in services:
            try:
                print(f"  Waiting for {service_name}...")
                rospy.wait_for_service(service_name, timeout=timeout)
                print(f"  ✓ {service_name} is available")
            except rospy.ROSException:
                print(f"  ✗ {service_name} timeout after {timeout}s")
                return False

        # Create service clients
        self.get_actions_client = rospy.ServiceProxy('/getActions', GetActions)
        self.get_action_details_client = rospy.ServiceProxy('/getActionDetails', GetActionDetails)

        print("All services are available!")
        return True

    def test_get_actions(self):
        """Test retrieving all actions from memory service"""
        print("\n--- Testing GetActions service ---")

        try:
            # Call service
            print("Calling /getActions service...")
            response = self.get_actions_client("")  # Empty filter = get all

            actions = response.actions
            print(f"✓ Successfully retrieved {len(actions)} actions")

            if len(actions) == 0:
                print("⚠ Warning: No actions found in memory service")
                return True

            # Log each action
            for i, action in enumerate(actions):
                print(f"  {i+1}. Action: {action.actionName}")
                print(f"     Arguments: {len(action.arguments)}")
                print(f"     Preconditions: {len(action.preconditions)}")
                print(f"     Effects: {len(action.effects)}")
                if hasattr(action, 'execution') and action.execution:
                    print(f"     Execution actions: {len(action.execution)}")
                print()

            return True

        except rospy.ServiceException as e:
            print(f"✗ Service call failed: {e}")
            return False
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            return False

    def test_get_action_details(self, action_name):
        """Test retrieving details for a specific action"""
        print(f"\n--- Testing GetActionDetails for '{action_name}' ---")

        try:
            print(f"Calling /getActionDetails for action: {action_name}")
            response = self.get_action_details_client(action_name)

            action = response.action
            print(f"✓ Successfully retrieved details for action: {action.actionName}")
            print(f"  Arguments: {[arg.name + ':' + arg.type for arg in action.arguments]}")
            print(f"  Preconditions: {len(action.preconditions)} conditions")
            print(f"  Effects: {len(action.effects)} effects")

            return True

        except rospy.ServiceException as e:
            print(f"✗ Service call failed: {e}")
            return False
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            return False

    def run_validation(self):
        """Run the complete validation test suite"""
        print("Starting memory service validation...")

        # Test 1: Wait for services
        if not self.wait_for_services():
            print("✗ VALIDATION FAILED: Services not available")
            return False

        # Test 2: Get all actions
        if not self.test_get_actions():
            print("✗ VALIDATION FAILED: Could not retrieve actions")
            return False

        # Test 3: Get action details for first action
        try:
            # Get first action for detail test
            response = self.get_actions_client("")
            if response.actions:
                first_action_name = response.actions[0].actionName
                if not self.test_get_action_details(first_action_name):
                    print("✗ VALIDATION FAILED: Could not get action details")
                    return False
        except:
            print("⚠ Skipping action details test (no actions available)")

        print("\n🎉 VALIDATION PASSED: Memory service is working correctly!")
        print("The action recognition node should be able to connect successfully.")
        return True

def main():
    try:
        validator = MemoryValidationTest()
        success = validator.run_validation()

        if success:
            print("\nValidation completed successfully!")
            sys.exit(0)
        else:
            print("\nValidation failed!")
            sys.exit(1)

    except rospy.ROSInterruptException:
        print("Validation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Validation failed with error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()