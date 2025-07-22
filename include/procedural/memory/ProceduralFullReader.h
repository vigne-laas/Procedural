#ifndef PROCEDURALFULLREADER_H
#define PROCEDURALFULLREADER_H
#include <string>
#include <tree/ParseTree.h>
#include "procedural/memory/FullParser.h"



namespace procedural {

class ProceduralFullReader {
public:
    ProceduralFullReader() = default;
    explicit ProceduralFullReader(const std::string& path);
    bool read(const std::string& path);
    Actions_t getActions() { return parser_.getActions(); }
    std::vector<PracticeFrame*> getPracticeFrames() { return parser_.getPracticeFrames(); }
    std::vector<Practice*> getPractices() { return parser_.getPractices(); }

private:
    antlr4::tree::ParseTree* tree{};
    FullParser parser_;

};

} // procedural

#endif //PROCEDURALFULLREADER_H
