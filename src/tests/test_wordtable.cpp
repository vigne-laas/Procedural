//
// Created by avigne on 18/01/23.
//

#include "test_wordtable.h"
#include <iostream>

int main() {
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
    Table.printAll();
    uint32_t index = Table["bonjour"];
    uint32_t index2 = Table["coucou"];
    std::cout << index << " " << index2 << std::endl;
    std::string txt = Table[0];
    std::string txt2 = Table[1];
    std::cout << txt << " " << txt2 << std::endl;
    Table.printAll();



    return 0;
}
