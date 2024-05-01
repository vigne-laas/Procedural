#ifndef PROCEDURAL_PARSEDARGS_H
#define PROCEDURAL_PARSEDARGS_H

#include <ostream>

namespace procedural {
struct ParsedArgs_t {
    ParsedArgs_t() : args() {}

    bool empty() const { return args.empty(); }

    friend std::ostream& operator<<(std::ostream& os, const ParsedArgs_t& lhs)
    {
        if (lhs.empty())
            return os;
        for (const auto& arg: lhs.args)
            os << arg.first << "(" << arg.second << ")" << "\n";
        return os;
    }

    int size() const { return args.size(); }

    std::map<std::string, std::string> args;

};
}
#endif //PROCEDURAL_PARSEDARGS_H
