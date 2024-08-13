#include "procedural/task_recognition/Builder/HTNBuilder.h"

namespace procedural {

bool HTNBuilder::build(const HTNParserd_t& htn)
{
  for (const auto& task: htn.tasks) {
       auto t = new Task(task.name);
       t->build(task);
       tasks_.push_back(t);
   }
    return true;

}

bool HTNBuilder::checkAction(const std::vector<Action*>& actions)
{
    return false;
}

bool HTNBuilder::build(const HTNParserd_t& htn, const std::vector<Action*>& actions)
{
     for (const auto& task: htn.tasks) {
       auto t = new Task(task.name);
       t->build(task);
       tasks_.push_back(t);
   }
    return true;
}
} // procedural