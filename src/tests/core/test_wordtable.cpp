#include "procedural/core/Types/WordTable.h"
#include <iostream>

int main()
{
    procedural::WordTable Table = procedural::WordTable();
    Table.add("coucou");
    //std::cout << "index : " << index << std::endl;
    std::unordered_set<std::string> list;
    list.insert("test");
    list.insert("ajout");
    list.insert("depuis");
    list.insert("une");
    list.insert("liste");
    list.insert("ajout");
    Table.add(list);
    Table.add("coucou");
    std::cout << Table.toString() << std::endl;
    uint32_t index = Table["bonjour"];
    uint32_t index2 = Table["coucou"];
    std::cout << index << " " << index2 << std::endl;
    std::string txt = Table[0];
    std::string txt2 = Table[1];
    std::cout << txt << " " << txt2 << std::endl;
    std::cout << Table.toString() << std::endl;

    return 0;
}
