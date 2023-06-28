#include <thread>
#include "ros/ros.h"
#include "procedural/RosInterface.h"
#include "std_msgs/String.h"


ros::NodeHandle* node_;
std::map<std::string, procedural::RosInterface*> interfaces_;
std::map<std::string, std::thread> interfaces_threads_;
OntologiesManipulator* onto_manipulators;
mementar::TimelinesManipulator* time_manipulators;


bool deleteInterface(const std::string& name)
{
    interfaces_[name]->stop();
    usleep(1000);
    try
    {
        interfaces_threads_[name].join();
    }
    catch(std::runtime_error& ex)
    {
        ROS_ERROR("Catch error when joining the interface thread : %s",std::string(ex.what()).c_str());
        ROS_WARN("The thread will be detached");
        interfaces_threads_[name].detach();
    }

    interfaces_threads_.erase(name);
    delete interfaces_[name];
    interfaces_.erase(name);

    std::cout << name << " STOPED" << std::endl;
    return true;
}



std::vector<std::string> parse_msg(const std::string& msg)
{
    std::regex regex("\\|");
    std::vector<std::string> out(
            std::sregex_token_iterator(msg.begin(), msg.end(), regex, -1),
            std::sregex_token_iterator()
    );
    return out;
}


void callback_manager(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "msg received : " << msg << std::endl;
    std::vector<std::string> params = parse_msg(msg->data);
    if(params[0]=="ADD")
    {
        auto it = interfaces_.find(params[1]);
        if(it != interfaces_.end())
            return;
        else
        {
            auto tmp = new procedural::RosInterface(node_,*onto_manipulators,*time_manipulators,params[1]);
            interfaces_[params[1]] = tmp;
            std::thread th(&procedural::RosInterface::run,tmp);
            interfaces_threads_[params[1]] = std::move(th);
            std::cout <<"ActionRecognition : " << params[1] << " STARTED" << std::endl;
        }
    }


}


int main(int argc, char** argv)
{
//    std::cout << "<<<<<<<<<<<<<<<<<< Launching Action Recognition <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    ros::init(argc, argv, "Action_Recognition_multi");
    ros::NodeHandle node;
    node_ = &node;
    onto_manipulators = new OntologiesManipulator(node_);
    time_manipulators = new mementar::TimelinesManipulator(node_);

    int buffer_max_size_ = 10;
    ros::Subscriber subscriber_action = node_->subscribe<std_msgs::String>("/new_human",buffer_max_size_,callback_manager);
    ros::spin();

    std::vector<std::string> interfaces_names;
    std::transform(interfaces_.cbegin(), interfaces_.cend(), std::back_inserter(interfaces_names), [](const auto& interface){ return interface.first; });

    for(size_t i = 0; i < interfaces_names.size(); i++)
        deleteInterface(interfaces_names[i]);

    ROS_DEBUG("KILL action recognition");


    return 0;
}
