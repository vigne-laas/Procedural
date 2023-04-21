#ifndef PROCEDURAL_ACTIONRECOGNITION_H
#define PROCEDURAL_ACTIONRECOGNITION_H

#include <vector>
#include <queue>
#include <mutex>
#include "procedural/core/Types/Fact.h"
#include "procedural/utils/BufferFacts.h"

namespace procedural {
    class Action;

    class ActionRecognition {
    public:
        explicit ActionRecognition(const std::vector<Action *>& actions,double tll = 25,int max_size=500);

        void addToQueue(Fact *fact);

        void processQueue(TimeStamp_t current_time);

    private:
        BufferFacts buffer_;
        std::vector<Action *> actions_;
    };

} // procedural

#endif //PROCEDURAL_ACTIONRECOGNITION_H
