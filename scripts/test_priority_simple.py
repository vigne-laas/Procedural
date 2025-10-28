#!/usr/bin/env python3

import rospy
import sys
from procedural_interfaces.srv import GetPriorities

def test_priority_parsing():
    rospy.init_node('test_priority_parsing', anonymous=True)

    # Wait for service to be available
    service_name = '/procedural/get_priorities'
    print(f"Waiting for service {service_name}...")
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        print(f"Service {service_name} not available - this test requires the procedural node to be running")
        print("Try: rosrun procedural procedural_node")
        return False

    try:
        # Call service to get priorities
        get_priorities = rospy.ServiceProxy(service_name, GetPriorities)
        response = get_priorities()

        print(f"Found {len(response.priorities)} priorities:")

        for priority in response.priorities:
            print(f"\n--- Priority: {priority.name} ---")
            print(f"Level: {priority.level}")
            print(f"Preconditions ({len(priority.preconditions)}):")
            for precond in priority.preconditions:
                print(f"  {precond}")

            print(f"Legacy objectives ({len(priority.objectives)}):")
            for obj in priority.objectives:
                print(f"  {obj}")

            print(f"State objectives ({len(priority.state_objectives)}):")
            for state_obj in priority.state_objectives:
                print(f"  {state_obj}")

            if priority.task_name:
                print(f"Task: {priority.task_name}")
                print(f"Task parameters ({len(priority.task_parameters)}):")
                for param in priority.task_parameters:
                    print(f"  {param}")

        return True

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

if __name__ == '__main__':
    success = test_priority_parsing()
    sys.exit(0 if success else 1)