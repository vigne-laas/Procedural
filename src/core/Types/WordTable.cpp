#include "procedural/core/Types/WordTable.h"

#include <algorithm>

namespace procedural {

WordTable::WordTable() : table_({""})
{}

std::string WordTable::toString()
{
    std::string str;
    for(size_t i = 0; i < table_.size(); i++)
        str += std::to_string(i) + " : " + table_[i] + "\n";
    return str;
}

void WordTable::add(const std::string& word)
{
    if (std::find(table_.begin(), table_.end(), word) == table_.end())
        table_.push_back(word);
}

void WordTable::add(const std::unordered_set<std::string>& words)
{
    for (const auto& word: words)
        add(word);
}

const std::string& WordTable::get(uint32_t index)
{
    if ((index > 0) && (index < table_.size()))
        return table_[index];
    else
        return table_.front();
}

uint32_t WordTable::get(const std::string& word)
{
    auto iter = std::find(table_.begin(), table_.end(), word);
    if (iter == table_.end())
    {
        add(word);
        return table_.size() - 1;
    } else
        return std::distance(table_.begin(), iter);
}

const std::string& WordTable::operator[](uint32_t index)
{
    return get(index);
}

uint32_t WordTable::operator[](const std::string& word)
{
    return get(word);
}

uint32_t WordTable::getConst(const std::string& word) const
{
    auto iter = std::find(table_.begin(), table_.end(), word);
    if (iter != table_.end())
        return std::distance(table_.begin(), iter);
    else
        return 0;
}

} // namespace procedural