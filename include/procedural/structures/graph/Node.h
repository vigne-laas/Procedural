#ifndef PROCEDURAL_NODE_H
#define PROCEDURAL_NODE_H


#include <string>
#include "Transition.h"

namespace procedural {
class Action;

class Node {
public:
    Node(uint64_t id, const std::string& name, int depth) : id_(id), name_(name), depth_(depth) {}

    virtual ~Node() {}

    uint64_t getId() const { return id_; }

    bool isFinal() const { return transitions_.empty(); }

    std::string getFullName() const { return name_ + "_" + std::to_string(id_); }

    const std::vector<std::shared_ptr<Transition>>& getTransitions() const { return transitions_; }

    const std::vector<uint64_t>& getParents() { return parents_; };

    int getDepth() const { return depth_; }

    std::string toString() const { return getFullName(); }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) { return os << node.toString(); }

    void addTransition(std::shared_ptr<Transition> transition)
    {
        transitions_.push_back(transition);
    }

    void addParent(uint64_t id_parent) { parents_.push_back(id_parent); }

    uint64_t match(const Observation* observation) const;

    std::string toDot(int current_node_id = 0) const;

    void completeRemap(const std::vector<std::shared_ptr<Action>>& actions);



private:
    uint64_t id_;
    std::string name_;
    int depth_;
    std::vector<std::shared_ptr<Transition>> transitions_;
    std::vector<uint64_t> parents_;

};

} // procedural

#endif //PROCEDURAL_NODE_H
