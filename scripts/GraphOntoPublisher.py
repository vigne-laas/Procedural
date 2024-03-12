#!/usr/bin/env python3
import rospy
import time

from mementar.srv import MementarServiceRequest
from ontologenius import Ontoros
from ontologenius import OntologiesManipulator
from mementar.srv import MementarService, MementarServiceRequest
from graphPublisher.execution.SequenceExecution import ActionExecutor
from ontologenius.msg import OntologeniusStampedString


class OntologyPublisher:
    def __init__(self, name=None):
        rospy.init_node("OntologyTaskPublisher", anonymous=False)

        ## onto part
        self.ontos_ = OntologiesManipulator()
        print("Waiting ontologenius")
        self.ontos_.waitInit()
        print("end of Wait ontologenius")
        self.initOntologenius(name)

        ## mementar_part
        print("Wait Mementar")
        rospy.wait_for_service("/mementar/manage")
        print("end of Wait /mementar/manage")
        self.mementar_manager = rospy.ServiceProxy("/mementar/manage", MementarService)

        self.initMementar(name)

        if name is not None:
            self.pub = rospy.Publisher("/ontologenius/insert_stamped/" + name, OntologeniusStampedString, queue_size=1)
            self.initOntologenius(name)
            self.initMementar(name)
            self.initRecognition(name)
        else:
            self.pub = rospy.Publisher("/ontologenius/insert_stamped", OntologeniusStampedString, queue_size=1)
        pass

    def initOntologenius(self, name):
        self.ontos_.add(name)
        self.ontos_.get(name).close()
        time.sleep(0.5)
        self.ontos_.get(name).feeder.waitConnected()
        pass

    def initMementar(self, name):
        req = MementarServiceRequest()
        req.action = "add"
        req.param = name
        res = self.mementar_manager.call(req)
        if res.code == 0:
            print("Add to mementar  ", name)
        else:
            print("Issue on add instance to mementar")

    def initRecognition(self, name):
        pass

    def sendSequence(self, sequence):
        for s in sequence:
            print(f'publish : {s}')
            stamp = Ontoros.getRosTime()
            print(stamp)
            msg = OntologeniusStampedString(data=s, stamp=stamp)
            self.pub.publish(msg)
            time.sleep(0.1)
            pass


# Utilisation de la classe
if __name__ == '__main__':
    publisher = OntologyPublisher("pr2")
    folder_path = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot'
    yaml_file = '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/tests/kitchen_domain/publisher.yaml'
    executor = ActionExecutor(yaml_file, folder_path)
    executor.set_callback(publisher.sendSequence)
    executor.execute_actions()
