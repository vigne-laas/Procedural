#include <std_msgs/String.h>
#include "procedural/RosInterface.h"
#include "mementar/ActionsPublisher.h"
#include <stdexcept>

namespace procedural {


RosInterface::RosInterface(ros::NodeHandle* n, const std::string& name) : node_(n), name_(name)
{
//    std::cout << ">>>>>>>>>>>>>>>>>>> Waiting ontology manipulator " << std::endl;
    manipulator_ = new OntologyManipulator(name_);
//    std::cout << ">>>>>>>>>>>>>>>>>>> Waiting ontology feeder Connected " << std::endl;
    manipulator_->close();
    //manipulator_->feeder.waitConnected();
    actions_publisher_ = new mementar::ActionsPublisher(n, name);
    std::cout << "<<<<<<<<<<<<<<<<<<< Init ok ontology manipulator " << std::endl;
    parse();
    build();
    link();
}

void RosInterface::run()
{
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        recognition_->processQueue({now.sec, now.nsec});
        ros::spinOnce();
        loop_rate.sleep();
    }

}

/// ------------------------------- Private PART ----------------------------------------------- ///
bool RosInterface::parse()
{
    std::string path;
    node_->param<std::string>("PATH_YAML", path,
                              "/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/src/reader/test.yaml");
    return reader_.read(path);
}

bool RosInterface::build()
{
//    std::cout << "simple action : " << std::endl;
//    for (auto& action: reader_.getSimpleActions())
//        std::cout << action << std::endl;
//    std::cout << "composed action : " << std::endl;
//    for (auto& action: reader_.getComposedActions())
//        std::cout << action << std::endl;
//    auto onto = manipulator_.get(name_);
//    objectClient_ = &(onto->objectProperties);

    builder_.build(reader_.getSimpleActions(), reader_.getComposedActions(), &(manipulator_->objectProperties));
    auto Actions_ = builder_.getActions();
//    for (auto action: Actions_)
//        for (auto sub_action: action->getSpecializedActions())
//            std::cout << sub_action.getStrStructure() << "\n\n" << std::endl;
//            std::cout << sub_action.toString() << std::endl;
    StateMachine::displayTypesTable();

    node_->param<double>("TTL_BUFFER", ttl_buffer_, 25);
    node_->param<int>("MAX_SIZE_BUFFER", buffer_max_size_, 500);
    recognition_ = new ActionRecognition(builder_.getActions(), ttl_buffer_, buffer_max_size_);
    return true;
}
bool RosInterface::link()
{
    feeder_.setCallback([&](procedural::Fact* fact) { recognition_->addToQueue(fact); });
    output_pub_ = node_->advertise<std_msgs::String>(getTopicName("outputNetwork"), 1);

    auto callback = [&](const mementar::StampedFact::ConstPtr& msg) { inputConverter(msg); };
    sub_input_stamped_facts_ = node_->subscribe<mementar::StampedFact>(getTopicName("inputFacts"), buffer_max_size_,
                                                                       callback);
    recognition_->setCallback([&](const std::vector<procedural::NetworkOutput>& outputs) {
        for (auto& output: outputs)
        { output_pub_.publish(outputConverter(output)); };
    });
    return false;
}

void RosInterface::OntologeniusPublisher(const NetworkOutput& output)
{
    ros::Time stamp_time((double) output.start_time.toFloat());
    if (manipulator_->individuals.exist(output.name) == false && manipulator_->classes.exist(output.type) == false)
        manipulator_->feeder.addInheritage(output.type + "Action","ActionType");
//        manipulator_->feeder.addInheritage("ActionType", output.type + "Action");

    for (auto& description: output.descriptions)
    {
        std::string subject;
        std::string object;
        if (description.var_subject_ == nullptr)
        {
            if (description.var_subject_str_ == "self")
                subject = output.name;
            else
                subject = description.var_subject_str_;
        } else
            subject = Fact::individuals_table[description.var_subject_->getValue()];

        if (description.var_object_ == nullptr)
        {
            if (description.var_object_str_ == "self")
                object = output.name;
            else
                object = description.var_object_str_;
        } else
            object = Fact::individuals_table[description.var_object_->getValue()];
        if(object.empty() == false and subject.empty() == false)
            manipulator_->feeder.addProperty(subject, description.property_, object);


    }
}

std_msgs::String RosInterface::outputConverter(const NetworkOutput& output)
{
    std::cout << " new output : " << output << std::endl;
    std_msgs::String res;
    std::stringstream ss;
    ss << output;
    res.data = ss.str();
    TimeStamp_t delta_t{0, 500000000};
    auto start_time(output.start_time.removeDeltaT(delta_t));
    auto stop_time(output.stop_time.addDeltaT(delta_t));


//    std::cout << "start time :" << start_time << std::endl;
//    std::cout << "stop time :" << stop_time << std::endl;

    actions_publisher_->insert(output.name, {(uint32_t) start_time.sec_, (uint32_t) start_time.nsec_},
                               {(uint32_t) stop_time.sec_, (uint32_t) stop_time.nsec_});
    OntologeniusPublisher(output);
//    usleep(500);
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
    return (onto_name.empty()) ? "/ActionRecognition/" + topic_name : "/ActionRecognition/" + topic_name + "/" +
                                                                      onto_name;
}


} // procedural