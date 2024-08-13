#include "procedural/task_recognition/Structures/Task.h"

namespace procedural {

  bool Task::build(const Abstract_task_t& task)
  {
    for(const auto& method : task.methods_)
    {
      auto *m = new Method(method, task.name, task.arguments, (int)factory_methods_.size());
      if(m->close())
      {
//        m->saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot");
        factory_methods_.push_back(m);
      }
      else
        throw std::runtime_error("Method not closed");
    }
    return true;
  }

  bool Task::evolve(Observation* obs)
  {
    uint match = 0;
    for(auto method_it = active_methods_.begin(); method_it != active_methods_.end(); ++method_it)
    {
//      LOG_DEBUG << "trying to evolve method: " << (*method_it)->getName();
      if((*method_it)->evolve(obs))
      {
//        LOG_DEBUG << "Evolved method: " << (*method_it)->getName();
        if((*method_it)->getState() > GraphState::Finished)
        {
          finished_methods_.push_back(*method_it);
          method_it = active_methods_.erase(method_it);
          method_it--;
        }
        match ++;
      }
    }
    if(match==0)
    {
      for (Method* factory_method  : factory_methods_)
      {
        Graph* graph_clone = (factory_method->clone(method_id_++));
        if(graph_clone == nullptr)
          throw std::runtime_error("Graph not cloned");

        if(graph_clone->evolve(obs))
        {
          LOG_DEBUG << "Evolve factory method: " << graph_clone->getName();

          active_methods_.push_back(static_cast<Method*>(graph_clone));
          match++;
        }
        else
          delete graph_clone;

      }
    }
    return match>0;
  }

} // namespace procedural