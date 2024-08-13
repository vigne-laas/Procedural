#ifndef PROCEDURAL_WORDTABLE_H
#define PROCEDURAL_WORDTABLE_H

#include <string>
#include <unordered_set>
#include <vector>

namespace procedural {

class WordTable {
public:
    WordTable() : table_({""}) {};

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

    std::vector<std::string>::iterator begin() {
        return table_.begin();
    }

    std::vector<std::string>::const_iterator begin() const {
        return table_.begin();
    }

    std::vector<std::string>::iterator end() {
        return table_.end();
    }

    std::vector<std::string>::const_iterator end() const {
        return table_.end();
    }


    static WordTable actions_table;
    static WordTable properties_table;
    static WordTable individuals_table;
    static WordTable types_table;

private:
    std::vector<std::string> table_;
};


} // namespace procedural

#endif //PROCEDURAL_WORDTABLE_H
