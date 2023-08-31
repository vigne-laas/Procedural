#include <std_msgs/String.h>
#include "procedural/RosInterface.h"
#include <stdexcept>

namespace procedural {


RosInterface::RosInterface(ros::NodeHandle* n, onto::OntologiesManipulator& onto_manipulators,
                           mementar::TimelinesManipulator& time_manipulators, const std::string& name) : node_(n),
                                                                                                         run_(true),
                                                                                                         name_(name)

{
    ROS_INFO("Action Recognition start : %s", name_.c_str());

    onto_manipulators.waitInit();
    onto_manipulators.add(name_);
    onto_manipulator_ = onto_manipulators.get(name_);
    onto_manipulator_->close();

    time_manipulators.waitInit();
    auto res = time_manipulators.add(name_);
//    std::cout << "resultat after all init add mementar : " << res << std::endl;
    timeline_manipulator_ = time_manipulators.get(name_);

    ROS_INFO("Action Recognition ready : %s", name_.c_str());
}

bool RosInterface::init(const std::string& descriptions_path, double ttl_buffer, int buffer_max_size)
{
    if(reader_.read(descriptions_path) == false)
        return false;

    ttl_buffer_ = ttl_buffer;
    buffer_max_size_ = buffer_max_size;

    build();
    link();

    return true;
}

void RosInterface::run()
{
    ros::Rate loop_rate(20);
    while (ros::ok() && isRunning())
    {
        ros::Time now = ros::Time::now();
        recognition_.processQueue({now.sec, now.nsec});
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/// ------------------------------- Private PART ----------------------------------------------- ///
void RosInterface::build()
{
    builder_.build(reader_.getSimpleActions(), reader_.getComposedActions(), onto_manipulator_);

    StateMachine::displayTypesTable();

    recognition_.init(builder_.getActions(), ttl_buffer_, buffer_max_size_);
}

void RosInterface::link()
{
    feeder_.setCallback([&](procedural::Fact* fact) { recognition_.addToQueue(fact); });
    output_pub_ = node_->advertise<std_msgs::String>(getTopicName("outputStateMachines"), 1);

    sub_input_stamped_facts_ = node_->subscribe<mementar::StampedFact>(getMementarTopicName(), buffer_max_size_,
                                                                       &RosInterface::inputConverter, this);
    recognition_.setCallback([&](const std::vector<procedural::StateMachineFinishedMSG_>& outputs) {
        for (auto& output : outputs)
            output_pub_.publish(outputConverter(output));
    });
}

void RosInterface::ontologeniusPublisher(const StateMachineFinishedMSG_& output)
{
    ros::Time stamp_time((double) output.start_time.toFloat());
    if (onto_manipulator_->individuals.exist(output.name) == false &&
        onto_manipulator_->classes.exist(output.type) == false)
        onto_manipulator_->feeder.addInheritage(output.type + "Action", "ActionMethod");

    for (auto& description : output.descriptions)
    {
        std::string subject;
        std::string object;
        if (description.var_subject_ == nullptr)
        {
            if (description.var_subject_str_ == "self")
                subject = output.name;
            else
                subject = description.var_subject_str_;
        }
        else
            subject = Fact::individuals_table[description.var_subject_->getValue()];

        if (description.var_object_ == nullptr)
        {
            if (description.var_object_str_ == "self")
                object = output.name;
            else
                object = description.var_object_str_;
        }
        else
            object = Fact::individuals_table[description.var_object_->getValue()];

        if (object.empty() == false and subject.empty() == false)
            onto_manipulator_->feeder.addProperty(subject, description.property_, object);
    }
}

std_msgs::String RosInterface::outputConverter(const StateMachineFinishedMSG_& output)
{
    std::cout << " new output : " << output << std::endl;
    std_msgs::String res;
    std::stringstream ss;
    ss << output;
    res.data = ss.str();
    TimeStamp_t delta_t{0, 500000000};
    auto start_time(output.start_time.removeDeltaT(delta_t));
    auto stop_time(output.stop_time.addDeltaT(delta_t));

    timeline_manipulator_->action_feeder.insert(output.name, {(uint32_t) start_time.sec_, (uint32_t) start_time.nsec_},
                                                {(uint32_t) stop_time.sec_, (uint32_t) stop_time.nsec_});
    ontologeniusPublisher(output);

    return res;
}
void RosInterface::inputConverter(const mementar::StampedFact::ConstPtr& msg)
{
    try
    {
        feeder_.feed(msg->added, msg->subject, msg->predicat, msg->object, std::stoi(msg->id),
                     {msg->stamp.sec, msg->stamp.nsec});
    }
    catch (const std::exception& e)
    {
//        std::cout << "invalid fact not added to feeder : " << e.what() << " msg : " << (*msg) << std::endl;
    }

}


std::string RosInterface::getTopicName(const std::string& topic_name)
{
    return getTopicName(topic_name, name_);
}

std::string RosInterface::getTopicName(const std::string& topic_name, const std::string& onto_name)
{
    return (onto_name.empty()) ? "/ActionRecognition/" + topic_name :
                                 "/ActionRecognition/" + topic_name + "/" + onto_name;
}
std::string RosInterface::getMementarTopicName()
{
    return (name_.empty()) ? "/mementar/echo" : "/mementar/echo/" + name_;
}

} // procedural