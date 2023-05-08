#include <std_msgs/String.h>
#include "procedural/RosInterface.h"


namespace procedural {


RosInterface::RosInterface(ros::NodeHandle* n, const std::string& name) : node_(n), name_(name)
{
    std::cout << ">>>>>>>>>>>>>>>>>>> Waiting ontology manipulator " << std::endl;
    if (name_.empty())
    {
        ros::service::waitForService("/ontologenius/object_property");
        manipulator_ = new OntologyManipulator(n);
    } else
    {
        auto manipulators = new OntologiesManipulator(n);
        manipulators->waitInit();
        manipulator_ = manipulators->get(name_);
    }
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
    for (auto action: Actions_)
        for (auto sub_action: action->getSpecializedActions())
            std::cout << sub_action.getStrStructure() << "\n\n" << std::endl;
//            std::cout << sub_action.toString() << std::endl;
    Network::displayTypesTable();

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

std_msgs::String RosInterface::outputConverter(const NetworkOutput& output)
{
    std::cout << " new output : " << output << std::endl;
    std_msgs::String res;
    std::stringstream ss;
    ss << output;
    res.data = ss.str();
    return res;
}
void RosInterface::inputConverter(const mementar::StampedFact::ConstPtr& msg)
{
    feeder_.feed(msg->added, msg->subject, msg->predicat, msg->object, std::stoi(msg->id),
                 {msg->stamp.sec, msg->stamp.nsec});
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