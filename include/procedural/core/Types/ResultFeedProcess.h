#ifndef PROCEDURAL_RESULTFEEDPROCESS_H
#define PROCEDURAL_RESULTFEEDPROCESS_H

namespace procedural {

enum class FeedResult
{
    NO_EVOLUTION = 0,
    EVOLVE = 2,
    FINISH = 3,


};

struct EvolveResult_t
{
    FeedResult state = FeedResult::NO_EVOLUTION;
    bool update_available = false;
};

struct ResultFeedProcess_t
{


    bool evolve = false;
    std::vector<Action*> finished_actions;
    std::vector<Action*> updated_actions;


    void combine(const EvolveResult_t& new_result, Action* action)
    {
        switch (new_result.state)
        {
            case FeedResult::NO_EVOLUTION:
                this->evolve |= false;
                break;
            case FeedResult::EVOLVE:
                this->evolve |= true;
                break;
            case FeedResult::FINISH:
                this->evolve |= true;
                this->finished_actions.push_back(action);
                break;
        }
        if (new_result.update_available)
            this->updated_actions.push_back(action);
    }
};
}

#endif //PROCEDURAL_RESULTFEEDPROCESS_H
