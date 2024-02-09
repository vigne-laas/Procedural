#include <thread>
#include "ros/ros.h"
#include "procedural/RosInterface.h"
#include "std_msgs/String.h"
#include "overworld/GetAgents.h"
#include "procedural/utils/Parameters.h"

ros::NodeHandle* node_;
std::map<std::string, procedural::RosInterface*> interfaces_;
std::map<std::string, std::thread> interfaces_threads_;
onto::OntologiesManipulator* onto_manipulators;
mementar::TimelinesManipulator* time_manipulators;

procedural::Parameters params;

bool deleteInterface(const std::string& name)
{
    interfaces_[name]->stop();
    usleep(1000);
    try
    {
        interfaces_threads_[name].join();
    }
    catch (std::runtime_error& ex)
    {
        ROS_ERROR("Catch error when joining the interface thread : %s", std::string(ex.what()).c_str());
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
    std::vector<std::string> msg_params = parse_msg(msg->data);
    if (msg_params[0] == "ADD")
    {
        auto it = interfaces_.find(msg_params[1]);
        if (it != interfaces_.end())
            return;
        else
        {
            auto tmp = new procedural::RosInterface(node_, *onto_manipulators, *time_manipulators, msg_params[1]);
            if (tmp->init(params.at("action_path").getFirst(), stod(params.at("ttl").getFirst()),
                          stoi(params.at("max_size").getFirst()), params.at("domain_path").getFirst()))
            {
                interfaces_[msg_params[1]] = tmp;
                std::thread th(&procedural::RosInterface::run, tmp);
                interfaces_threads_[msg_params[1]] = std::move(th);
                std::cout << "ActionRecognition : " << msg_params[1] << " STARTED" << std::endl;
            }
        }
    }
}

int main(int argc, char** argv)
{
    std::cout << "<<<<<<<<<<<<<<<<<< Launching Action Recognition <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    ros::init(argc, argv, "Action_Recognition_multi");
    ros::NodeHandle node;
    node_ = &node;
    onto_manipulators = new onto::OntologiesManipulator();
    time_manipulators = new mementar::TimelinesManipulator(node_);

    params.insert(procedural::Parameter("action_path", {"-a", "--action_path"}));
    params.insert(procedural::Parameter("ttl", {"-t", "--ttl"}, {"25"}));
    params.insert(procedural::Parameter("max_size", {"-s", "--max_size"}, {"500"}));
    params.insert(procedural::Parameter("domain_path", {"-d", "--domain_path"}, {""}));

//    params.insert(procedural::Parameter("name", {"-n", "--name"}, {"pepper"}));
//
    params.set(argc, argv);
    params.display();

//    auto tmp = new procedural::RosInterface(node_, *onto_manipulators, *time_manipulators, params.at("name").getFirst());
//    if(tmp->init(params.at("yaml_path").getFirst(), stod(params.at("ttl").getFirst()), stoi(params.at("max_size").getFirst())))
//    {
//        interfaces_[params.at("name").getFirst()] = tmp;
//        std::thread th(&procedural::RosInterface::run, tmp);
//        interfaces_threads_[params.at("name").getFirst()] = std::move(th);
//        std::cout <<"ActionRecognition : " << params.at("name").getFirst() << " STARTED" << std::endl;
//    }

    ros::Subscriber subscriber_action = node_->subscribe<std_msgs::String>("/overworld/new_assessor", 10,
                                                                           callback_manager);

    ros::ServiceClient client = node_->serviceClient<overworld::GetAgents>("/overworld/getAgents");
    overworld::GetAgents srv;
    if (client.call(srv))
        for (auto agent: srv.response.agents)
        {
            auto it = interfaces_.find(agent);
            if (it == interfaces_.end())
            {
                auto tmp = new procedural::RosInterface(node_, *onto_manipulators, *time_manipulators, agent);
                if (tmp->init(params.at("action_path").getFirst(), stod(params.at("ttl").getFirst()),
                              stoi(params.at("max_size").getFirst()), params.at("domain_path").getFirst()))
                {
                    interfaces_[agent] = tmp;
                    std::thread th(&procedural::RosInterface::run, tmp);
                    interfaces_threads_[agent] = std::move(th);
                    std::cout << "ActionRecognition : " << agent << " STARTED" << std::endl;
                }
            }
        }


    ros::spin();

    std::vector<std::string> interfaces_names;
    std::transform(interfaces_.cbegin(), interfaces_.cend(), std::back_inserter(interfaces_names),
                   [](const auto& interface) { return interface.first; });

    for (auto& interfaces_name: interfaces_names)
        deleteInterface(interfaces_name);

    ROS_DEBUG("KILL action recognition");

    return 0;
}
