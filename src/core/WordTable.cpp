#include "procedural/core/WordTable.h"
#include <iostream>
#include <algorithm>

namespace procedural {

WordTable::WordTable() : table_({""})
{
}

void WordTable::printSize()
{
    std::cout << "Size of the table : " << table_.size() << std::endl;
}

void WordTable::printAll()
{
    std::cout << "Elements in  the table : " << std::endl;
    for (unsigned int i = 0; i < table_.size(); i++)
    {
        std::cout << "table[" << i << "] : " << table_[i] << std::endl;
    }
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
    if (index > 0 & index < table_.size())
        return table_[index];
    else
        return table_[0];
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

uint32_t WordTable::get_const(const std::string& word) const
{
    auto iter = std::find(table_.begin(), table_.end(), word);
    if (iter != table_.end())
        return std::distance(table_.begin(), iter);
    else
        return 0;
}

} // procedural