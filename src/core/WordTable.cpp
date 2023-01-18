#include "procedural/core/WordTable.h"
#include <iostream>
#include <algorithm>

namespace procedural {

WordTable::WordTable() : table_({"null"})
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

void WordTable::add(const std::string &word)
{
    if (std::find(table_.begin(), table_.end(), word) == table_.end())
        table_.push_back(word);

}

void WordTable::add(std::unordered_set<std::string> set)
{
    for (const auto &elemt: set)
    {
        add(elemt);
    }

}

std::string &WordTable::get(uint32_t index)
{
    return table_[index];
}

uint32_t WordTable::get(const std::string &word)
{
    auto iter = std::find(table_.begin(), table_.end(), word);
    if (iter == table_.end())
    {
        add(word);
        return table_.size() - 1;
    } else
    {
        return std::distance(table_.begin(), iter);
    }
}

const std::string &WordTable::operator[](uint32_t index) const
{
    return table_[index];
}

std::string &WordTable::operator[](uint32_t index)
{
    return table_[index];
}

uint32_t WordTable::operator[](std::string word)
{
    return get(word);
}

} // procedural