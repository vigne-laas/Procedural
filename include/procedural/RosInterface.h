#ifndef PROCEDURAL_ROSINTERFACE_H
#define PROCEDURAL_ROSINTERFACE_H

#include <ros/ros.h>

#include "procedural/reader/YamlReader.h"
#include "procedural/reader/ActionBuilder.h"
#include "procedural/feeder/Feeder.h"
#include "procedural/utils/ActionRecognition.h"
#include "procedural/core/Types/StateMachineOutput.h"

#include "mementar/StampedFact.h"
#include "mementar/ActionsPublisher.h"

#include "ontologenius/clients/ontologyClients/ObjectPropertyClient.h"
#include "ontologenius/OntologiesManipulator.h"

#include "std_msgs/String.h"


namespace procedural {

class RosInterface
{
public:
    explicit RosInterface(ros::NodeHandle* n, const std::string& name = "");

    void run();

private:
    bool parse();
    bool build();
    bool link();

    std_msgs::String outputConverter(const StateMachineOutput& output);
    void inputConverter(const mementar::StampedFact::ConstPtr& msg);
    void OntologeniusPublisher(const StateMachineOutput& output);



    ros::NodeHandle* node_;
    ros::Publisher output_pub_;
    ros::Subscriber sub_input_stamped_facts_;

    OntologyManipulator* manipulator_;
    mementar::ActionsPublisher* actions_publisher_;
//    ObjectPropertyClient* objectClient_;



    std::string name_;

    YamlReader reader_;
    ActionBuilder builder_;
    ActionRecognition* recognition_;
    Feeder feeder_;

    double ttl_buffer_;
    int buffer_max_size_;

    std::string getTopicName(const std::string& topic_name);
    std::string getTopicName(const std::string& topic_name, const std::string& onto_name);
};

} // procedural

#endif //PROCEDURAL_ROSINTERFACE_H
