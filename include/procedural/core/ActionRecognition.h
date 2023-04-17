#ifndef PROCEDURAL_ACTIONRECOGNITION_H
#define PROCEDURAL_ACTIONRECOGNITION_H

#include <vector>
#include <queue>
#include <mutex>
#include "procedural/core/Types/Fact.h"

namespace procedural {
    class Action;

    class ActionRecognition {
    public:
        explicit ActionRecognition(std::vector<Action *> *actions);

        void addToQueue(Fact *fact);

        void processQueue();

    private:
        std::mutex mutex_lock;

        std::queue<Fact *> primary_queue;
        std::queue<Fact *> secondary_queue;
        std::queue<Fact *> *read_queue;
        std::queue<Fact *> *write_queue;
        std::vector<Action *> *actions_;
    };

} // procedural

#endif //PROCEDURAL_ACTIONRECOGNITION_H
