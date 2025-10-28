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
        // Utilise directement toRosMsg() qui gère correctement les champs literal/value
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
bool MemoryROSInterface::getRoles(procedural_interfaces::getRoles::Request& req,
    procedural_interfaces::getRoles::Response& res)
{
    auto roles_ptrs = parser_.getRoles();
    res.roles.clear();
    res.roles.reserve(roles_ptrs.size());
    for (const auto* role_ptr : roles_ptrs)
    {
        if (role_ptr != nullptr)
        {
            res.roles.push_back(*role_ptr);
        }
    }
    return true;
}

bool MemoryROSInterface::getPriorities(procedural_interfaces::GetPriorities::Request& req,
    procedural_interfaces::GetPriorities::Response& res)
{
    auto priorities_ptrs = parser_.getPriorities();
    res.priorities.clear();
    res.priorities.reserve(priorities_ptrs.size());
    for (const auto* priority_ptr : priorities_ptrs)
    {
        if (priority_ptr != nullptr)
        {
            res.priorities.push_back(*priority_ptr);
        }
    }
    return true;
}

bool MemoryROSInterface::getTasks(procedural_interfaces::GetTasks::Request& req,
    procedural_interfaces::GetTasks::Response& res)
{
    auto tasks = parser_.getTasks();
    res.tasks.tasks.clear();
    res.tasks.tasks.reserve(tasks.size());

    for (const auto& task: tasks)
    {
        if (req.filter.empty() || task.name.find(req.filter) != std::string::npos)
        {
            res.tasks.tasks.push_back(task.toRosMsg());
        }
    }
    std::cout << "GetTasks service called - returned " << res.tasks.tasks.size() << " tasks" << std::endl;
    return true;
}

bool MemoryROSInterface::getActions(procedural_interfaces::GetActions::Request& req,
    procedural_interfaces::GetActions::Response& res)
{
    // Use existing getRobotActions functionality but with the new message format
    auto actions = parser_.getActions().actions;
    res.actions.clear();
    res.actions.reserve(actions.size());

    for (const auto& action: actions)
    {
        if (req.filter.empty() || action.name.find(req.filter) != std::string::npos)
        {
            procedural_interfaces::Action action_msg = action.toRosMsg();
            res.actions.push_back(action_msg);
        }
    }
    std::cout << "GetActions service called - returned " << res.actions.size() << " actions" << std::endl;
    return true;
}

bool MemoryROSInterface::getTaskDetails(procedural_interfaces::GetTaskDetails::Request& req,
    procedural_interfaces::GetTaskDetails::Response& res)
{
    auto tasks = parser_.getTasks();
    res.found = false;

    for (const auto& task: tasks)
    {
        if (task.name == req.task_name)
        {
            res.task = task.toRosMsg();
            res.found = true;
            break;
        }
    }
    std::cout << "GetTaskDetails service called for task: " << req.task_name
              << " - " << (res.found ? "found" : "not found") << std::endl;
    return true;
}

bool MemoryROSInterface::getActionDetails(procedural_interfaces::GetActionDetails::Request& req,
    procedural_interfaces::GetActionDetails::Response& res)
{
    auto actions = parser_.getActions().actions;
    res.found = false;

    for (const auto& action: actions)
    {
        if (action.name == req.action_name)
        {
            res.action = action.toRosMsg();
            res.found = true;
            break;
        }
    }
    std::cout << "GetActionDetails service called for action: " << req.action_name
              << " - " << (res.found ? "found" : "not found") << std::endl;
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
    // getRoles_service_ = node_->advertiseService("/getRoles", &MemoryROSInterface::getRoles, this);
    // std::cout << "Service getRoles started" << std::endl;
    getPriorities_service_ = node_->advertiseService("/getPriorities", &MemoryROSInterface::getPriorities, this);
    std::cout << "Service getPriorities started" << std::endl;
    getTasks_service_ = node_->advertiseService("/getTasks", &MemoryROSInterface::getTasks, this);
    std::cout << "Service getTasks started" << std::endl;
    getActions_service_ = node_->advertiseService("/getActions", &MemoryROSInterface::getActions, this);
    std::cout << "Service getActions started" << std::endl;
    getTaskDetails_service_ = node_->advertiseService("/getTaskDetails", &MemoryROSInterface::getTaskDetails, this);
    std::cout << "Service getTaskDetails started" << std::endl;
    getActionDetails_service_ = node_->advertiseService("/getActionDetails", &MemoryROSInterface::getActionDetails, this);
    std::cout << "Service getActionDetails started" << std::endl;
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
