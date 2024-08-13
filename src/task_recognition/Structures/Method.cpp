#include "procedural/task_recognition/Structures/Method.h"

#include <queue>

namespace procedural {
  Method::Method(const Method_t& abstract_method,
                 const std::string& name, const std::vector<Arguments_t>& vector, int id)
    : Graph(name + "_m_" + std::to_string(id), id, name)
  {
    LOG_INFO << "Construction Method: " << name;
    LOG_INFO << "Method ID: " << id;
    auto subtask = abstract_method.subtask;
    subtask.linkActions();
    std::vector<Ordered_Action_t> actions_to_build;
    actions_to_build.reserve(subtask.map_actions.size());
    for(const auto& task : subtask.map_actions)
    {
      //        LOG_INFO << "SubTask: " << task.first << " : " << task.second;
      actions_to_build.push_back(task.second);
    }
    std::sort(actions_to_build.begin(), actions_to_build.end(),
              [](const Ordered_Action_t& a, const Ordered_Action_t& b) {
                return a.after_id.size() < b.after_id.size();
              });
    int current_id = 0;
    int loop_id = 0;
    while(!actions_to_build.empty())
    {
      int nb_update = 0;
      for(auto it = actions_to_build.begin(); it != actions_to_build.end(); it++)
      {
        LOG_INFO << "Action to build: " << *it;
        auto* obs = new Observation(*it);
        auto transition = std::make_shared<Transition>(current_id * 10, obs, (current_id + 1) * 10, it->id);
        //            LOG_INFO << "Initial Transition: " << transition->toString();
        if(Method::addTransition(transition))
        {
          it = actions_to_build.erase(it);
          it--;
          nb_update++;
        }
        current_id++;
      }
      if(nb_update == 0 && !actions_to_build.empty())
      {
        LOG_ERROR << "Loop detected";
        for(const auto& action : actions_to_build)
        {
          LOG_ERROR << "Action: " << action;
        }
        saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/builder_task/" + name + "_m_eror" +
                std::to_string(id) + ".dot");
        break;
      }
      loop_id++;
    }
    saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/builder_task/" + name + "_m_" +
            std::to_string(id) + ".dot");
  }

  bool Method::addTransition(std::shared_ptr<Transition> transition)
  {
    if(state_ != GraphState::UnClosed)
      return false;
    if(nodes_.find(transition->getSourceId()) == nodes_.end())
    {
      addNode(transition->getSourceId());
    }
    else
    {
      auto node = nodes_[transition->getSourceId()];
      if(!node->satisfyTransitionConstraint(transition))
      {
        LOG_ERROR << "Constraint not satisfied";
        return false;
      }
    }
    if(nodes_.find(transition->getTargetId()) == nodes_.end())
    {
      addNode(transition->getTargetId());
    }

    table_variables_.set(transition->getTableVariables());
    if(nodes_[transition->getSourceId()]->addTransition(transition))
    {
      nodes_[transition->getTargetId()]->addSatisfyConstraint(nodes_[transition->getSourceId()]->getSatisfyConstraint());
      nodes_[transition->getTargetId()]->addSatisfyConstraint(transition->getHtnId());
      nodes_[transition->getTargetId()]->addParent(transition->getSourceId());
    }

    if(transition->getSourceId() == 0)
    {
      //        LOG_INFO << "Source == 0 ";
      return true;
    }

    // Déclaration de la variable pour stocker le noeud
    auto satisfied_nodes = findSatisfiedNodes(transition);

    auto ordered_nodes = std::vector<std::shared_ptr<Node>>(satisfied_nodes.begin(), satisfied_nodes.end());
    std::sort(ordered_nodes.begin(), ordered_nodes.end(),
              [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
                return a->getDepth() < b->getDepth();
              });

    for(const auto& unsatisfied_node : ordered_nodes)
    {
      replicateTransitions(unsatisfied_node, nodes_[transition->getSourceId()], nodes_[transition->getTargetId()]);
    }

    return true;
  }

  std::vector<std::shared_ptr<Transition>>
  Method::getPathTransitions(std::shared_ptr<Node> source, std::shared_ptr<Node> target)
  {
    std::unordered_map<uint64_t, std::shared_ptr<Transition>> cameFrom;
    std::queue<std::shared_ptr<Node>> queue;
    queue.push(source);

    while(!queue.empty())
    {
      auto current = queue.front();
      queue.pop();

      if(current->getId() == target->getId())
      {
        std::vector<std::shared_ptr<Transition>> path;
        while(current->getId() != source->getId())
        {
          auto transition = cameFrom[current->getId()];
          path.push_back(transition);
          current = nodes_[transition->getSourceId()];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      for(const auto& transition : current->getTransitions())
      {
        if(cameFrom.find(transition->getTargetId()) == cameFrom.end())
        {
          queue.push(nodes_[transition->getTargetId()]);
          cameFrom[transition->getTargetId()] = transition;
        }
      }
    }

    return {}; // Return empty vector if no path is found
  }

  void Method::DFS(std::shared_ptr<Node> source, std::shared_ptr<Node> target, std::vector<std::shared_ptr<Transition>>& path,
                   std::set<uint64_t>& visitedTransitions, std::vector<std::vector<std::shared_ptr<Transition>>>& allPaths)
  {
    if(source->getId() == target->getId())
    {
      allPaths.push_back(path);
      return;
    }

    for(const auto& transition : source->getTransitions())
    {
      if(visitedTransitions.find(transition->getId()) == visitedTransitions.end())
      {
        visitedTransitions.insert(transition->getId());
        path.push_back(transition);
        DFS(nodes_[transition->getTargetId()], target, path, visitedTransitions, allPaths);
        path.pop_back();
        visitedTransitions.erase(transition->getId());
      }
    }
  }

  std::vector<std::vector<std::shared_ptr<Transition>>>
  Method::getAllPaths(std::shared_ptr<Node> source, std::shared_ptr<Node> target)
  {
    std::vector<std::vector<std::shared_ptr<Transition>>> allPaths;
    std::vector<std::shared_ptr<Transition>> path;
    std::set<uint64_t> visitedTransitions;
    DFS(source, target, path, visitedTransitions, allPaths);
    return allPaths;
  }

  void Method::replicateTransitions(std::shared_ptr<Node> source1, std::shared_ptr<Node> target1,
                                    std::shared_ptr<Node> target2)
  {
    std::shared_ptr<Transition> t = nullptr;
    for(const auto& trans : target1->getTransitions())
    {
      if(trans->getTargetId() == target2->getId())
      {
        t = trans;
      }
    }
    auto temp_node_id = getMaxIdAtDepth(source1->getDepth() + 1) + 1;
    //    LOG_INFO << "Temp id: " << temp_node_id;
    auto search_id_direct = getIdAtDepthThatSatisfyConstraint(source1->getDepth() + 1, t,
                                                              source1->getSatisfyConstraint());
    //    LOG_INFO << "Search id direct : " << search_id_direct;
    if(search_id_direct > 0)
    {
      temp_node_id = search_id_direct;
      //        LOG_DEBUG << "Node direct already exists with id: " << temp_node_id;
    }
    addNode(temp_node_id);
    auto new_transition = std::make_shared<Transition>(source1->getId(), new Observation(*t->getObservation()),
                                                       temp_node_id, t->getHtnId());
    //    LOG_INFO << "New direct transition: " << new_transition->toString();
    if(source1->addTransition(new_transition))
    {
      nodes_[temp_node_id]->addParent(source1->getId());
      nodes_[temp_node_id]->addSatisfyConstraint((int)t->getHtnId());
      nodes_[temp_node_id]->addSatisfyConstraint(source1->getSatisfyConstraint());
    }

    auto source_id = temp_node_id;
    //    saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/builder_task/" + getName() +
    //            +"_source_" + std::to_string(source1->getId()) + "_direct.dot");

    auto vect_transitions = getAllPaths(source1, target1);
    auto path_id = 0;
    for(const auto& path : vect_transitions)
    {
      //        LOG_INFO << ">>>>>>>>>>>>>> Path to replicate: " << path_id++;
      auto step = 0;
      source_id = temp_node_id;
      for(const auto& transition : path)
      {
        //            LOG_INFO << "Transition: " << transition->toString();
        auto temp_id = 0;
        if(nodes_[source_id]->getDepth() + 1 == target2->getDepth())
        {
          temp_id = target2->getId();
          //                LOG_DEBUG << "Node targets with id: " << temp_id;
        }
        else
        {
          temp_id = getMaxIdAtDepth(nodes_[source_id]->getDepth() + 1) + 1;
          //                LOG_INFO << "Temp id: " << temp_id;
          //                LOG_DEBUG << "Wanted node with id: " << temp_id << " at depth: " << nodes_[source_id]->getDepth() + 1;
          auto search_id = getIdAtDepthThatSatisfyConstraint(nodes_[source_id]->getDepth() + 1, transition,
                                                             nodes_[source_id]->getSatisfyConstraint());
          //                LOG_INFO << "Search id replication: " << search_id;
          if(search_id > 0)
          {
            temp_id = search_id;
            //                    LOG_DEBUG << "Node already exists with id: " << temp_id;
          }
        }

        addNode(temp_id);

        new_transition = std::make_shared<Transition>(nodes_[source_id]->getId(),
                                                      new Observation(*transition->getObservation()), temp_id,
                                                      transition->getHtnId());
        //            LOG_INFO << "New transition: " << new_transition->toString();
        if(nodes_[source_id]->addTransition(new_transition))
        {
          nodes_[temp_id]->addParent(source_id);
          nodes_[temp_id]->addSatisfyConstraint((int)transition->getHtnId());
          nodes_[temp_id]->addSatisfyConstraint(nodes_[source_id]->getSatisfyConstraint());
        }

        //        LOG_DEBUG << "Transition between " << source_id << "(" << nodes_[source_id]->toString() << ") and " << temp_id
        //                  << " ( " << nodes_[temp_id]->toString() << ") added";
        source_id = temp_id;
        //            saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/builder_task/" + getName() +
        //                    +"_source_" + std::to_string(source1->getId()) + "_path_" + std::to_string(path_id) + "_step_" +
        //                    std::to_string(step) + ".dot");
        step++;
      }
      //        saveDot("/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/builder_task/" + getName() +
      //                "_path_" +
      //                std::to_string(path_id) + ".dot");
    }

    //    auto transitions = getPathTransitions(source1, target1);
    //    if (transitions.empty()) {
    //        LOG_ERROR << "No path found";
    //    }
    //    LOG_INFO << ">>>>>>>>>>>>>> Transitions to replicate: ";
  }

  std::set<std::shared_ptr<Node>>
  Method::findSatisfiedNodes(std::shared_ptr<Transition> transition)
  {
    //    LOG_DEBUG << "Find satisfied nodes for transition: " << transition->toString();
    //    LOG_DEBUG << "After id: ";
    //    for (const auto& id: transition->getObservation()->getConstraints()) {
    //        LOG_DEBUG << id;
    //    }
    //    LOG_DEBUG << "\n";
    std::queue<uint64_t> queue;
    std::unordered_set<uint64_t> visited;
    std::set<std::shared_ptr<Node>> satisfiedNodes;

    queue.push(transition->getSourceId());
    visited.insert(transition->getSourceId());

    while(!queue.empty())
    {
      uint64_t current_id = queue.front();
      queue.pop();
      auto current_node = nodes_[current_id];

      for(const auto& parent_id : current_node->getParents())
      {
        if(visited.find(parent_id) == visited.end())
        {
          auto parent_node = nodes_[parent_id];
          if(parent_node->satisfyTransitionConstraint(transition))
          {
            satisfiedNodes.insert(parent_node);
          }
          queue.push(parent_id);
          visited.insert(parent_id);
        }
      }
    }

    // Include the source node if it satisfies the constraints
    //    if (checkConstraints(after_id, nodes_[transition->getSourceId()]->getSatisfyConstraint())) {
    //        satisfiedNodes.insert(nodes_[transition->getSourceId()]);
    //    }

    return satisfiedNodes;
  }

  bool Method::checkConstraints(const std::set<int>& constraints, const std::unordered_set<int>& set) const
  {
    return std::all_of(constraints.begin(), constraints.end(), [&set](const auto& constraint) {
      return set.find(constraint) != set.end();
    });
  }

} // namespace procedural