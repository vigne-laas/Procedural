#ifndef PROCEDURAL_WORDTABLE_H
#define PROCEDURAL_WORDTABLE_H

#include <string>
#include <unordered_set>
#include <vector>

namespace procedural {

class WordTable
{
public:
    WordTable();
    ~WordTable() = default;

    void printSize();
    void printAll();

    void add(const std::string& word);
    void add(const std::unordered_set<std::string>& list);

    const std::string& get(uint32_t index);
    uint32_t get(const std::string& word);

    uint32_t get_const(const std::string& word) const;

    const std::string& operator[](uint32_t index);
    uint32_t operator[](const std::string& word);

private:
    std::vector<std::string> table_;
};

} // procedural

#endif //PROCEDURAL_WORDTABLE_H
