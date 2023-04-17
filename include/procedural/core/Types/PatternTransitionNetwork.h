#ifndef PROCEDURAL_PATTERNTRANSITIONNETWORK_H
#define PROCEDURAL_PATTERNTRANSITIONNETWORK_H

#include <map>
#include <string>

namespace procedural {

struct PatternTransitionNetwork_t
{
    PatternTransitionNetwork_t(int origin, int next,
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

#endif // PROCEDURAL_PATTERNTRANSITIONNETWORK_H
