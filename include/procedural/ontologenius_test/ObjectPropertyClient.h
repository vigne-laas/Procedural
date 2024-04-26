#ifndef PROCEDURAL_OBJECTPROPERTYCLIENT_H
#define PROCEDURAL_OBJECTPROPERTYCLIENT_H

#include <string>
#include <vector>

namespace onto
{
class ObjectPropertyClient
{
public:
    ObjectPropertyClient(const std::string &name) : name_(name) {}
    virtual ~ObjectPropertyClient() = default;
    virtual std::vector<std::string> getDown(const std::string &property, int depth=-1) = 0;
    std::string name_;
};
}// namespace onto
#endif //PROCEDURAL_OBJECTPROPERTYCLIENT_H
