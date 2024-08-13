#ifndef PROCEDURAL_METHOD_H
#define PROCEDURAL_METHOD_H

#include "procedural/structures/graph/Graph.h"
#include "procedural/task_recognition/Reader/domainTypes/ParsedHTN.h"

namespace procedural {

  class Method : public Graph
  {
  public:
    Method(const Method_t& abstract_method, const std::string& string, const std::vector<Arguments_t>& vector, int id);

    bool addTransition(std::shared_ptr<Transition> transition) override;

    bool checkConstraints(const std::set<int>& constraints, const std::unordered_set<int>& set) const;

    std::set<std::shared_ptr<Node>> findSatisfiedNodes(std::shared_ptr<Transition> transition);

    std::vector<std::shared_ptr<Transition>>
    getPathTransitions(std::shared_ptr<Node> source, std::shared_ptr<Node> target);

    void replicateTransitions(std::shared_ptr<Node> source1, std::shared_ptr<Node> target1, std::shared_ptr<Node> target2);

    std::vector<std::vector<std::shared_ptr<Transition>>> getAllPaths(std::shared_ptr<Node> source, std::shared_ptr<Node> target);

    void DFS(std::shared_ptr<Node> source, std::shared_ptr<Node> target, std::vector<std::shared_ptr<Transition>>& path,
             std::set<uint64_t>& visitedTransitions, std::vector<std::vector<std::shared_ptr<Transition>>>& allPaths);


  };

} // namespace procedural

#endif // PROCEDURAL_METHOD_H
