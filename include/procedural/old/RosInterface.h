#ifndef PROCEDURAL_ROSINTERFACE_H
#define PROCEDURAL_ROSINTERFACE_H

#include <ros/ros.h>

#include "procedural/old/reader/YamlReader.h"
#include "procedural/old/builder/ActionBuilder.h"
#include "procedural/old/feeder/Feeder.h"
#include "procedural/old/core/ActionRecognition.h"
#include "procedural/old/core/Types/StateMachineFinishedMSG_.h"

#include "ontologenius/OntologiesManipulator.h"

#include "mementar/TimelinesManipulator.h"
#include "mementar/StampedFact.h"
#include "procedural/old/core/TaskRecognition.h"
#include "procedural/old/builder/HTNBuilder.h"
#include "procedural/old/reader/DomainReader.h"


namespace procedural {

class RosInterface
{
public:
    RosInterface(ros::NodeHandle* n, onto::OntologiesManipulator& onto_manipulators,
                 mementar::TimelinesManipulator& time_manipulators, const std::string& name = "");

    bool
    init(const std::string& descriptions_path, double ttl_buffer, int buffer_max_size, const std::string& domain_path);
    void run();
    void stop() { run_ = false; }
    inline bool isRunning() const { return run_; }

private:
    ros::NodeHandle* node_;
    onto::OntologyManipulator* onto_manipulator_;
    mementar::TimelineManipulator* timeline_manipulator_;

    bool run_;

    std::string name_;
    double ttl_buffer_;
    int buffer_max_size_;

    ros::Publisher output_pub_;
    ros::Subscriber sub_input_stamped_facts_;

    YamlReader reader_;
    ActionBuilder builder_;
    ActionRecognition recognition_;
    DomainReader domain_reader_;
    HTNBuilder htn_builder_;
    TaskRecognition task_recognition_;
    Feeder feeder_;

    void build();
    void link();

    std_msgs::String outputConverter(const StateMachineFinishedMSG_& output);
    std_msgs::String outputTaskConverter(const TaskRecognizedMSG_t& output);

    void inputConverter(const mementar::StampedFact::ConstPtr& msg);
    void ontologeniusPublisher(const StateMachineFinishedMSG_& output);

    std::string getTopicName(const std::string& topic_name);
    std::string getTopicName(const std::string& topic_name, const std::string& onto_name);
    std::string getMementarTopicName();
    bool task_recognition_enable_ = true;
};

} // procedural

#endif //PROCEDURAL_ROSINTERFACE_H
