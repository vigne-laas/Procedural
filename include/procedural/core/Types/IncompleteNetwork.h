#ifndef PROCEDURAL_INCOMPLETENETWORK_H
#define PROCEDURAL_INCOMPLETENETWORK_H

#include <map>
#include <string>


namespace procedural
{
class Network;
struct IncompleteNetwork_t
{
    IncompleteNetwork_t(Network* net,const std::map<std::string,std::string>& remap):network_(net),remap_variables_(remap){};

    Network * network_;
    std::map<std::string,std::string> remap_variables_;
};
}
#endif //PROCEDURAL_INCOMPLETENETWORK_H
