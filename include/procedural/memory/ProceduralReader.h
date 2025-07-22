#ifndef PROCEDURAL_PROCEDURAL_READER_H
#define PROCEDURAL_PROCEDURAL_READER_H

#include <string>
#include <vector>
#include <tree/ParseTree.h>

#include "ProceduralParser.h"
// #include "procedural/structures/procedural_structures/action_t.h"
#include "procedural_interfaces/action_t.h"
using namespace procedural_interfaces;

namespace procedural {
class ProceduralReader
{
public:
    ProceduralReader() = default;
    explicit ProceduralReader(const std::string& path);
    bool read(const std::string& path);
    std::vector<Action_t>& getActions() { return memory_; };

private:
    std::vector<Action_t> memory_;
    antlr4::tree::ParseTree* tree{};
    ProceduralParser listener;
};
} // procedural

#endif //PROCEDURAL_PROCEDURAL_READER_H
