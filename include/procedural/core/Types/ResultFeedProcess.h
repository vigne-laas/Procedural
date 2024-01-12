#ifndef PROCEDURAL_RESULTFEEDPROCESS_H
#define PROCEDURAL_RESULTFEEDPROCESS_H

namespace procedural {
class StateMachine;

enum class FeedResult
{
    NO_EVOLUTION = 0,
    NEW_EXPLANATION = 1,
    EVOLVE = 2,
    FINISH = 3,
};

struct ActionEvent_t
{
    ActionEvent_t(FeedResult state,StateMachine* sm) : state_(state), sm_finish(sm) {}

    FeedResult state_ = FeedResult::NO_EVOLUTION;
    StateMachine* sm_finish = nullptr;
};

struct EvolveResult_t
{
    FeedResult state = FeedResult::NO_EVOLUTION;
    bool update_available = false;
};
template<typename T>
struct ResultFeedProcess_t
{


    bool evolve = false;
    std::vector<T*> finished_;
    std::vector<T*> updated_;


    void combine(const EvolveResult_t& new_result, T* action)
    {
        switch (new_result.state)
        {
            case FeedResult::NO_EVOLUTION:
                this->evolve |= false;
                break;
            case FeedResult::NEW_EXPLANATION:
                this->updated_.push_back(action);
                this->evolve |= true;
                break;
            case FeedResult::EVOLVE:
                this->evolve |= true;
                break;
            case FeedResult::FINISH:
                this->evolve |= true;
                this->finished_.push_back(action);
                break;

        }
        if (new_result.update_available)
            this->updated_.push_back(action);
    }
};
}

#endif //PROCEDURAL_RESULTFEEDPROCESS_H
