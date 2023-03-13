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
    Network(const std::string& name, int id);
    Network(const Network& other) = delete;

    bool evolve(Fact* fact);

    const State* getCurrentState(){ return current_state_; }
    std::string getName(){ return full_name_; }

    bool isComplete(){ return current_state_->isFinalNode(); }
    bool isClosed(){ return closed_; }
    bool isValid(){ return valid_; }
    uint32_t getAge() { return age_; }

    bool addTransition(const PatternTransition_t& pattern);
    bool addDescription(const ActionDescription_t& des);

    bool closeNetwork();

    std::string toString();

    Network* clone(int new_id);
    void displayVariables();
    std::string describe(bool expl = false);
    std::vector<uint32_t> getIdsFacts(){ return id_facts_involve; };

    bool involveFacts(const std::set<uint32_t>& facts);

private:

    void addState(int id_state);
    void linkNetwork();
    void insertVariable(const std::string& variable);

    void processInitialState();



    std::string name_;
    uint32_t id_;
    std::string full_name_;

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
