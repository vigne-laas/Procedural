#ifndef PROCEDURAL_RESULTFEEDPROCESS_H
#define PROCEDURAL_RESULTFEEDPROCESS_H

namespace procedural
{

enum class FeedResult
{
    NO_EVOLUTION = 0,
    EVOLVE=2,
    FINISH=3

};

struct EvolveResult_t
{
    FeedResult state= FeedResult::NO_EVOLUTION;
    bool update_available= false;
};

struct ResultFeedProcess_t
{


    bool evolve = false;
    std::vector<ActionMethod*> finished_actions;
    std::vector<ActionMethod*> updated_actions;


    void combine(const ResultFeedProcess_t& new_result)
    {
        this->evolve |= new_result.evolve;
        this->finished_actions.insert(this->finished_actions.end(),new_result.finished_actions.begin(),new_result.finished_actions.end());
        this->updated_actions.insert(this->updated_actions.end(), new_result.updated_actions.begin(), new_result.updated_actions.end());
    }
};
}

#endif //PROCEDURAL_RESULTFEEDPROCESS_H
