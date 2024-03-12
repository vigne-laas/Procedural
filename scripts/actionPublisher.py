#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ontologenius.msg import OntologeniusStampedString, OntologeniusTimestamp


def release(A, S, O):
    sequence = [f"[ADD]{A}|hasHandMovingToward|{S}", f"[DEL]{A}|isHolding|{O}"]
    return sequence


def grasp(A, C):
    sequence = [f"[ADD]{A}|hasHandMovingToward|{C}", f"[ADD]{A}|isHolding|{C}"]
    return sequence


def pick_in(A, O, C):
    grasp_sequence = grasp(A, O)
    grasp_sequence.append(f"[DEL]{O}|inIn|{C}")
    return grasp_sequence


def pick_over(A, O, S):
    grasp_sequence = grasp(A, O)
    grasp_sequence.append(f"[DEL]{O}|isOnTopOf|{S}")
    return grasp_sequence


def place_in(A, O, S):
    release_sequence = release(A, S, O)
    release_sequence.append(f"[ADD]{O}|isInContainer|{S}")
    return release_sequence


def place_over(A, O, S):
    release_sequence = release(A, S, O)
    release_sequence.append(f"[ADD]{O}|isOnTopOf|{S}")
    return release_sequence


def give(A, A2, C):
    grasp_sequence = grasp(A, C)
    grasp_sequence.append(f"[DEL]{A2}|isHolding|{C}")
    return grasp_sequence


def split_eggs(Y, W, EY, EW, E, A):
    instructions = [
        f"[ADD]{EY}|isIn|{Y}",
        f"[ADD]{EW}|isIn|{W}",
        f"[ADD]{A}|isHolding|{E}"
    ]
    return instructions


def dispose(A, B, S):
    instructions = [
        f"[ADD]{A}|hasInHand|{B}",
        f"[ADD]{S}|hasSpreadOver|{B}",
        f"[DEL]{B}|hasIn|~"
    ]
    return instructions


def get(A, O, N):
    instructions = None
    if N == 0:
        instructions = pi


def poor_batter(A, M, B):
    instructions = [
        f"[ADD]{A}|hasInHand|{M}",
        f"[ADD]{B}|hasIn|{M}",
        f"[DEL]{M}|hasIn|~"
    ]
    return instructions


def poor_mold(A, P, M):
    instructions = [
        f"[ADD]{A}|hasInHand|{P}",
        f"[ADD]{M}|hasIn|{P}",
        f"[DEL]{M}|hasIn|~"
    ]
    return instructions


def cut(A, K, I):
    instructions = [
        f"[ADD]{A}|hasInHand|{K}",
        f"[ADD]{I}|isUnder|{K}",
        f"[ADD]{I}|isCutBy|{K}"
    ]
    return instructions


###### ----------------------------------------------- TASK part -------------------------------------------------------------------


def get(A, O, subtask_index, C=None, T=None, B=None):
    instructions = []
    if subtask_index == 1:
        instructions += pick(A, O)
    elif subtask_index == 2:
        instructions += place(A, C, T)
        instructions += pick(A, O)
    elif subtask_index == 3:
        instructions += get(B, O)
        instructions += give(B, A, O)
    elif subtask_index == 4:
        instructions += place(A, C, T)
        instructions += get(B, O)
        instructions += give(B, A, O)

    return instructions


def place(A, O, T, subtask_index):
    instructions = []
    # Subtask 1: Get the object
    instructions += get(A, O, 1)
    # Subtask 2: Place the object over or in the table
    if subtask_index == 1:
        instructions += place_over(A, O, T)
    elif subtask_index == 2:
        instructions += place_in(A, O, T)

    return instructions


def pick(A, O, subtask_index):
    # Choose subtask based on the provided index
    if subtask_index == 1:
        return pick_over(A, O)
    elif subtask_index == 2:
        return pick_in(A, O)
    else:
        raise ValueError("Invalid subtask index")


def mix(A, I, C, F):
    instructions = []

    # Subtask 1: Pick the ingredient
    instructions += pick(A, I)

    # Subtask 2: Place the ingredient in the container
    instructions += place(A, I, C, 2)

    # Subtask 3: Pick the whisk
    instructions += pick(A, F)

    # Subtask 4: Twist the whisk in the container
    instructions += twist(A, F, C, 3, 4)

    return instructions


def cut_over(A, I, K, S):
    instructions = []

    # Subtask 1: Pick the ingredient
    instructions += pick(A, I)

    # Subtask 2: Place the ingredient on the support
    instructions += place(A, I, S, 2)

    # Subtask 3: Pick the knife
    instructions += pick(A, K)

    # Subtask 4: Cut the ingredient using the knife
    instructions += cut(A, K, I, 3, 4)

    return instructions


def spread_batter(A, B, P):
    instructions = []

    # Subtask 1: Pick the pan
    instructions += pick(A, P)

    # Subtask 2: Place the pan on the support
    instructions += place(A, P, S, 2)

    # Subtask 3: Pick the batter
    instructions += pick(A, B)

    # Subtask 4: Place the batter in the pan
    instructions += place(A, B, P, 3, 4)

    return instructions


def fill(A, C, P):
    instructions = []

    # Subtask 1: Pour batter or mold into the container
    if C.isA == 'Batter':
        instructions += poor_batter(A, P, C)
    elif C.isA == 'Mold':
        instructions += poor_mold(A, P, C)

    return instructions


def bake(A, S, Hoven):
    instructions = []

    # Subtask 1: Place the support in the oven
    instructions += place(A, S, Hoven, 1)

    return instructions


def main():
    pub = rospy.Publisher('/ontologenius/insert/pr2', String, queue_size=1)
    pubStamped = rospy.Publisher('/ontologenius/insert_stamped/pr2', OntologeniusStampedString, queue_size=2)
    rospy.init_node('test_recognition_publisher')
    for s in pick_over('Bastien', 'Cube', 'Table'):
        # t = rospy.get_rostime()
        # time = OntologeniusTimestamp(t.secs, t.nsecs)
        msg = OntologeniusStampedString(data=s, stamp=rospy.get_rostime())
        pubStamped.publish(msg)
        input("wait to publish")


if __name__ == '__main__':
    main()
