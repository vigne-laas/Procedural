#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ontologenius.msg import StampedString


def talker():
    map_action = {
        "I": ["[ADD]cube_0|isA|Cube", "[ADD]cube_1|isA|Cube", "[ADD]table_0|isA|Support"],
        "G": ["[ADD]Bob|hasRightHandMovingToward|cube_0", "[ADD]Bob|hasInRightHand|cube_0"],
        "R": ["[ADD]Bob|hasLeftHandMovingToward|table_0", "[DEL]Bob|hasInLeftHand|cube_0"],
        "Pick": ["[ADD]Bob|hasHandMovingToward|cube_0", "[ADD]Bob|isHolding|cube_0", "[DEL]cube_0|isOnTopOf|table_0"],
        "Place": ["[ADD]Bob|hasHandMovingToward|table_0", "[ADD]cube_0|isOnTopOf|table_0", "[DEL]Bob|isHolding|cube_0"],
        "Place2": ["[ADD]cube_0|isOnTopOf|table_0", "[DEL]Bob|isHolding|cube_0"],
        "S": [["[ADD]Bob|hasRightHandMovingToward|cube_0", "[ADD]Bob|hasLeftHandMovingToward|cube_1"],
              ["[ADD]Bob|hasInRightHand|cube_0", "[ADD]Bob|hasInLeftHand|cube_1"],
              ["[DEL]cube_0|isOnTopOf|table_0", "[DEL]cube_1|isOnTopOf|table_0"]],
    }
    pub = rospy.Publisher('/ontologenius/insert/pr2', String, queue_size=1)
    pubStamped = rospy.Publisher('/ontologenius/insert_stamped/pr2', StampedString, queue_size=2)
    rospy.init_node('test_recognition_publisher')
    pub.publish("[ADD]cube_0|isA|Cube")
    keys = list(map_action.keys())
    while not rospy.is_shutdown():
        print("type possibles : ", keys)
        type = input("type : ")
        if type in keys:
            # print("type : ",type)
            if (type != "S"):
                print(map_action[type])
                for fact in map_action[type]:
                    input("wait to publish")
                    print('publish : ', fact)
                    pub.publish(fact)
            else:
                print(map_action[type])
                for facts in map_action[type]:
                    input("wait to publish")
                    print('publishs : ', facts)
                    time = rospy.get_rostime()
                    msg1 = StampedString(data=facts[0], stamp=time)
                    msg2 = StampedString(data=facts[1], stamp=time)
                    pubStamped.publish(msg1)
                    pubStamped.publish(msg2)
        elif type == "q":
            break
        else:
            continue


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
