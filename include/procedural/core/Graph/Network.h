#ifndef PROCEDURAL_NETWORK_H
#define PROCEDURAL_NETWORK_H

#include <unordered_set>
#include <unordered_map>
#include <map>
#include <exception>
#include <vector>
#include <string>
#include <set>

#include "procedural/core/Graph/State.h"
#include "procedural/core/Types/PatternTransition.h"
#include "procedural/core/Types/Description.h"
#include "procedural/core/Types/ActionDescription.h"
#include "procedural/core/Types/PatternNetworkTransition.h"

namespace procedural {

struct NetworkException : public std::exception
{
    std::string msg_;
    explicit NetworkException(const std::string& msg) : msg_(msg){}
    const char* what() const throw()
    {
        return msg_.c_str();
    }
};

struct NoInitialStateNetworkException : public NetworkException
{
    NoInitialStateNetworkException() : NetworkException("Invalid Network due to no initial State detected"){};
};

struct MultiInitialStateNetworkException : public NetworkException
{
    explicit MultiInitialStateNetworkException(const std::unordered_set<State*>& invalid_states) :
             NetworkException("Invalid Network due to no initial State detected\nState detected as initial are : ")
    {
        for (auto& state: invalid_states)
            msg_ += state->toString() + "\n";
    }
};

class Network
{
public:
    Network(const std::string& name, int id,uint32_t level=0);
    Network(const Network& other) = delete;

    bool evolve(Fact* fact);
    bool checkSubAction(Network* net);

    const State* getCurrentState() const { return current_state_; }
    std::string getTypeStr() const { return type_str_; }
    uint32_t getType() const { return type_; }
    std::string getName() const { return full_name_; }
    uint32_t getLevel() const { return level_;}

    bool isComplete() const { return current_state_->isFinalNode(); }
    bool isClosed() const { return closed_; }
    bool isValid() const { return valid_; }
    uint32_t getAge() const { return age_; }
    Variable_t getVar(const std::string& key) const { return variables_.at(key); }

    bool addTransition(const PatternTransition_t& pattern);
    bool addNetwork(const PatternNetworkTransition_t& network);
    bool addDescription(const ActionDescription_t& description);

    bool closeNetwork();

    std::string toString();

    Network* clone(int new_id);
    void displayVariables();
    std::string describe(bool expl = false);
    std::vector<uint32_t> getIdsFacts() const { return id_facts_involve; }

    bool involveFacts(const std::set<uint32_t>& facts);

    static WordTable types_table;

private:

    void addState(int id_state);
    void linkNetwork();
    void insertVariable(const std::string& variable);

    void processInitialState();

    std::string type_str_;
    uint32_t type_;
    uint32_t id_;
    std::string full_name_;
    uint32_t level_;

    std::map<std::string, Variable_t> variables_;

    std::vector<Description_t> descriptions_;

    State* current_state_;
    std::map<int, State*> states_;
    int id_initial_state_;

    std::vector<uint32_t> id_facts_involve;

    bool closed_;
    bool valid_;
    uint32_t age_;
};

} // procedural

#endif //PROCEDURAL_NETWORK_H
