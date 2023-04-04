#ifndef PROCEDURAL_PATTERNNETWORKTRANSITION_H
#define PROCEDURAL_PATTERNNETWORKTRANSITION_H

#include <map>
#include <string>
namespace procedural {

class NetworkTransition;

struct PatternNetworkTransition_t
{
    PatternNetworkTransition_t(int origin, const std::string& typeNetwork, int next,
                               const std::map<std::string, std::string>& remap_var) : origin_(origin),
                                                                                      next_(next),
                                                                                      remap_var_(remap_var)
    {
        type_ = NetworkTransition::subNetworkTable.get(typeNetwork);
    }

    int origin_;
    int next_;
    uint32_t type_;
    std::map<std::string, std::string> remap_var_;
};
}

#endif //PROCEDURAL_PATTERNNETWORKTRANSITION_H
