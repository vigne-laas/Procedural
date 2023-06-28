#ifndef PROCEDURAL_ROSINTERFACE_H
#define PROCEDURAL_ROSINTERFACE_H

#include <ros/ros.h>

#include "procedural/reader/YamlReader.h"
#include "procedural/reader/ActionBuilder.h"
#include "procedural/feeder/Feeder.h"
#include "procedural/utils/ActionRecognition.h"
#include "procedural/core/Types/StateMachineOutput.h"

#include "ontologenius/OntologiesManipulator.h"

#include "mementar/TimelinesManipulator.h"
#include "mementar/StampedFact.h"


namespace procedural {

class RosInterface
{
public:
    explicit RosInterface(ros::NodeHandle* n, OntologiesManipulator& onto_manipulators,
                          mementar::TimelinesManipulator& time_manipulators, const std::string& name = "");

    void run();
    void stop() { run_ = false; }
    inline bool isRunning() const { return run_; }

private:
    bool run_;

    bool parse();
    bool build();
    bool link();

    std_msgs::String outputConverter(const StateMachineOutput& output);
    void inputConverter(const mementar::StampedFact::ConstPtr& msg);
    void OntologeniusPublisher(const StateMachineOutput& output);


    ros::NodeHandle* node_;
    ros::Publisher output_pub_;
    ros::Subscriber sub_input_stamped_facts_;

    OntologyManipulator* onto_manipulator_;
    mementar::TimelineManipulator* timeline_manipulator_;

    std::string name_;

    YamlReader reader_;
    ActionBuilder builder_;
    ActionRecognition* recognition_;
    Feeder feeder_;

    double ttl_buffer_;
    int buffer_max_size_;

    std::string getTopicName(const std::string& topic_name);
    std::string getTopicName(const std::string& topic_name, const std::string& onto_name);
    std::string getMementarTopicName();
};

} // procedural

#endif //PROCEDURAL_ROSINTERFACE_H
