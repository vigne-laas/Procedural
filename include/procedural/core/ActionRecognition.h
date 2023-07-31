#ifndef PROCEDURAL_ACTIONRECOGNITION_H
#define PROCEDURAL_ACTIONRECOGNITION_H

#include <vector>
#include <queue>
#include <mutex>
#include "procedural/core/Types/Fact.h"
#include "procedural/utils/BufferFacts.h"
#include "procedural/core/Types/StateMachineFinishedMSG_.h"

namespace procedural {

class ActionType;

class ActionRecognition
{
public:

    ActionRecognition():buffer_(nullptr){};
    void init(const std::vector<ActionType*>& actions, double tll = 25, int max_size = 500);


    void addToQueue(Fact* fact) const;

    void processQueue(TimeStamp_t current_time);

    void setCallback(const std::function<void(const std::vector<StateMachineFinishedMSG_>&)>& callback)
    { callback_output_ = callback; }

private:
    static void defaultCallback(const std::vector<StateMachineFinishedMSG_>& outputs);
    BufferFacts* buffer_{};
    std::vector<ActionType*> actions_;

    std::function<void(const std::vector<StateMachineFinishedMSG_>&)> callback_output_;
};

} // namespace procedural

#endif //PROCEDURAL_ACTIONRECOGNITION_H
