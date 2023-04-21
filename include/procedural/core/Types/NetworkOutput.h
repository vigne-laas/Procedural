#ifndef PROCEDURAL_NETWORKOUTPUT_H
#define PROCEDURAL_NETWORKOUTPUT_H

#include "procedural/core/Graph/Network.h"
namespace procedural {

struct NetworkOutput
{
    explicit NetworkOutput(Network* net, bool update = false) : descriptions(net->getDescription()), updated(update),
                                                                start_time(net->getAge().toFloat()),
                                                                stop_time(net->getLastupdate().toFloat()),
                                                                name(net->getName())
    {};
    friend std::ostream& operator<<(std::ostream& os, const NetworkOutput& val)
    {
        if (val.updated)
            os << "new explanation for : ";
        else
            os << "finished : ";
        os << val.name << " [" << val.start_time << ";" << val.stop_time << "] ";
        if (val.descriptions.empty() == false)
        {
            os << " descriptions : ";
            for (auto& description: val.descriptions)
                os << description << " | ";
        }

        return os;
    }

    std::vector <std::string> descriptions;
    bool updated;
    float start_time;
    float stop_time;
    std::string name;
};
} //procedural
#endif //PROCEDURAL_NETWORKOUTPUT_H
