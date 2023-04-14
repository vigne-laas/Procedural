#ifndef PROCEDURAL_PATTERNNETWORKTRANSITION_H
#define PROCEDURAL_PATTERNNETWORKTRANSITION_H

#include <map>
#include <string>

namespace procedural {

struct PatternNetworkTransition_t
{
    PatternNetworkTransition_t(int origin, int next,
                               const std::string& network_type,
                               const std::map<std::string, std::string>& remap_var) : origin_(origin),
                                                                                      next_(next),
                                                                                      type_(network_type),
                                                                                      remap_var_(remap_var)
    {}

    int origin_;
    int next_;
    std::string type_;
    std::map<std::string, std::string> remap_var_;
};

} // namespace procedural

#endif // PROCEDURAL_PATTERNNETWORKTRANSITION_H
