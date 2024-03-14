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

    size_t size() { return table_.size(); }
    std::string toString();

    void add(const std::string& word);
    void add(const std::unordered_set<std::string>& list);

    const std::string& get(uint32_t index);
    uint32_t get(const std::string& word);

    uint32_t getConst(const std::string& word) const;

    const std::string& operator[](uint32_t index);
    uint32_t operator[](const std::string& word);

private:
    std::vector<std::string> table_;
};

} // namespace procedural

#endif //PROCEDURAL_WORDTABLE_H
