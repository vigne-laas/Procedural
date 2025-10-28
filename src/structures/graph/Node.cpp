#include "procedural/structures/graph/Node.h"

namespace procedural {

  uint64_t Node::match(const Observation* observation) const
  {
    for(const auto& transition : transitions_)
    {
      uint64_t target_id = transition->match(observation);
      if(target_id != 0)
      {
        return target_id;
      }
    }
    return 0;
  }

  std::string Node::toDot(int current_node_id) const
  {
    std::string shape;
    std::string fillcolor = "";
    std::string style = "";

    if(id_ == current_node_id)
    {
      shape = "box";
      fillcolor = ", fillcolor=lightblue, style=filled";
    }
    else if(id_ == 0)
    { // initial node
      shape = "ellipse";
      fillcolor = ", fillcolor=lightgreen, style=filled";
    }
    else if(isFinal())
    {
      shape = "doublecircle";
      fillcolor = ", fillcolor=lightcoral, style=filled";
    }
    else
    {
      shape = "circle";
    }
    // Convertir les contraintes satisfaites en une chaîne
    std::string satisfiedConstraints;
    for(const auto& constraint : getSatisfyConstraint())
    {
      satisfiedConstraints += std::to_string(constraint) + ",";
    }
    // Supprimer la dernière virgule
    if(!satisfiedConstraints.empty())
    {
      satisfiedConstraints.pop_back();
    }

    std::string label = getFullName() + "\\nDepth: " + std::to_string(depth_);
    if(!satisfiedConstraints.empty())
    {
      label += "\\nConstraints: " + satisfiedConstraints;
    }

    std::string result = "\"" + std::to_string(id_) + "\" [shape=" + shape + fillcolor + ", label=\"" + label + "\"];\n";

    for(const auto& transition : transitions_)
    {
      result += transition->toDot();
    }
    return result;
  }

  void Node::completeRemap(const std::vector<std::shared_ptr<Action>>& actions)
  {
    for(const auto& transition : transitions_)
    {
      transition->completeRemap(actions);
    }
  }

  void Node::addSatisfyConstraint(int id)
  {
    //    LOG_INFO << " add Satisfy constraint: " << id;
    set_satisfy_constraint_.insert(id);
    //    LOG_INFO << "node :: " << toString();
  }

  void Node::addSatisfyConstraint(const std::set<int>& ids)
  {
    //    LOG_INFO << " add Satisfy constraint set size: " << ids.size();
    set_satisfy_constraint_.insert(ids.begin(), ids.end());
    //    LOG_INFO << "node :: " << toString();
  }

  std::string Node::toString() const
  {
    std::string result = "Node: " + name_ + "_" + std::to_string(id_) + " Depth: " + std::to_string(depth_) + "\n";
    //    for (const auto& transition: transitions_) {
    //        result += transition->toString();
    //    }
    for(const auto& constraint : set_satisfy_constraint_)
    {
      result += "Satisfy Constraint: " + std::to_string(constraint) + "\n";
    }
    return result;
  }

  bool Node::satisfyTransitionConstraint(std::shared_ptr<Transition> transition) const
  {
    const auto& transitionConstraints = transition->getObservation()->getConstraints();
    if(transitionConstraints.empty())
    {
      return true;
    }

    // Check if all constraints of the transition are satisfied by the node
    auto res = std::includes(set_satisfy_constraint_.begin(), set_satisfy_constraint_.end(),
                             transitionConstraints.begin(), transitionConstraints.end());
    return res;
  }

  bool Node::addTransition(std::shared_ptr<Transition> transition)
  {
    auto res = std::find_if(transitions_.begin(), transitions_.end(),
                            [&transition](const std::shared_ptr<Transition>& t) {
                              return t->getTargetId() == transition->getTargetId();
                            });
    if(res == transitions_.end())
    {
      transitions_.push_back(transition);
      return true;
    }
    else
    {
      return false;
    }
  }

} // namespace procedural
