#ifndef PROCEDURAL_WORDTABLE_H
#define PROCEDURAL_WORDTABLE_H

#include <vector>
#include <string>
#include <unordered_set>

namespace procedural {

class WordTable
{
public:
    WordTable();
    void printSize();
    void printAll();
    void add(const std::string& word);
    void add(std::unordered_set<std::string> list); // pb avec const std::string &  et const set
    std::string& get(uint32_t index);
    uint32_t get(const std::string& word);
    const std::string& operator[](uint32_t index) const;
    std::string& operator[](uint32_t index);
    uint32_t operator[](std::string word);

private:
    std::vector<std::string> table_;
};

} // procedural

#endif //PROCEDURAL_WORDTABLE_H
