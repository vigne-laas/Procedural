#include "procedural/memory/ROS_Interfaces/MemoryRosInterface.h"
#include <procedural/memory/ProceduralFullReader.h>

namespace procedural {
MemoryROSInterface::MemoryROSInterface(ros::NodeHandle* n, const std::string& path)
{
    node_ = n;
    // parser_ = ProceduralReader();
    parser_ = ProceduralFullReader();
    parser_.read(path);
    init_ros();
    run_ = true;
}

bool MemoryROSInterface::getRobotActions(procedural_interfaces::getRobotActions::Request& req,
                                         procedural_interfaces::getRobotActions::Response& res)
{
    for (const auto& action: parser_.getActions().actions)
    {
        procedural_interfaces::Action action_msg;
        action_msg.actionName = action.name;
        for (const auto& [type, varname,value]: action.arguments)
        {
            procedural_interfaces::Argument arg_msg;
            arg_msg.type = type;
            arg_msg.value = varname;
            action_msg.arguments.push_back(arg_msg);
        }
        for (const auto& exec_action: action.executions_bloc)
        {
            procedural_interfaces::ExecutionAction exec_action_msg;
            exec_action_msg.name = exec_action.name;
            for (const auto& exec_arg: exec_action.arguments)
            {
                procedural_interfaces::ExecutionArgument exec_arg_msg;
                exec_arg_msg.name = exec_arg.type;
                exec_arg_msg.value = exec_arg.value;
                for (const auto& [key, value]: exec_arg.json)
                {
                    procedural_interfaces::JsonPair json_pair;
                    json_pair.key = key;
                    json_pair.value = value;
                    exec_arg_msg.json.push_back(json_pair);
                }
                exec_action_msg.arguments.push_back(exec_arg_msg);
            }
            action_msg.executionActions.push_back(exec_action_msg);
        }


        res.actions.push_back(action.toRosMsg());
    }
    return true;
}
bool MemoryROSInterface::getPractices(procedural_interfaces::getPractices::Request& req,
    procedural_interfaces::getPractices::Response& res)
{
    auto practices_ptrs = parser_.getPractices();
    res.practices.clear();
    res.practices.reserve(practices_ptrs.size());

    for (const auto* practice_ptr : practices_ptrs)
    {
        if (practice_ptr != nullptr)
        {
            res.practices.push_back(*practice_ptr);
        }
    }

    return true;
}
bool MemoryROSInterface::getPracticeFrames(procedural_interfaces::getPracticeFrames::Request& req,
    procedural_interfaces::getPracticeFrames::Response& res)
{
    auto practice_frames_ptrs = parser_.getPracticeFrames();
    res.practice_frames.clear();
    res.practice_frames.reserve(practice_frames_ptrs.size());
    for (const auto* frame_ptr : practice_frames_ptrs)
    {
        if (frame_ptr != nullptr)
        {
            res.practice_frames.push_back(*frame_ptr);
        }
    }
    return true;
}

void MemoryROSInterface::init_ros()
{
    getRobotActions_service_ = node_->advertiseService("/getRobotActions", &MemoryROSInterface::getRobotActions, this);
    std::cout << "Service getRobotActions started" << std::endl;
    getPractices_service_ = node_->advertiseService("/getPractices", &MemoryROSInterface::getPractices, this);
    std::cout << "Service getPractices started" << std::endl;
    getPracticeFrames_service_ = node_->advertiseService("/getPracticeFrames", &MemoryROSInterface::getPracticeFrames, this);
    std::cout << "Service getPracticeFrames started" << std::endl;
    std::cout << "ROS interface initialized" << std::endl;
}

void MemoryROSInterface::run() const
{
    ros::Rate loop_rate(20);
    while (ros::ok() && isRunning())
    {
        // ros::Time now = ros::Time::now();
        // recognition_.processQueue({now.sec, now.nsec});
        ros::spinOnce();
        loop_rate.sleep();
    }
}
} // procedural
