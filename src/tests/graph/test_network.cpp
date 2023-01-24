#include "test_network.h"
#include "procedural/core/FactPattern.h"
#include <iostream>

int main()
{
    std::vector<std::vector<procedural::FactPattern>> list_facts;
    procedural::FactPattern F = procedural::FactPattern(false, "A", "MoveThrought", "O", false);
    procedural::FactPattern F1 = procedural::FactPattern(false, "A", "hasInHand", "O", false);
    procedural::FactPattern F2 = procedural::FactPattern(true, "O", "overSupport", "S", true);
    procedural::FactPattern F3 = procedural::FactPattern(true, "O", "inContainer", "S", true);
//    std::cout << "Before Network \n"<< std::endl;
//    std::cout << F.to_string() << std::endl;
//    std::cout << F1.to_string() << std::endl;
//    std::cout << F2.to_string() << std::endl;
//    std::cout << F3.to_string() << std::endl;
//    std::cout << "END Before Network \n"<< std::endl;

    list_facts.resize(3);
    list_facts[0].push_back(F);
    list_facts[1].push_back(F1);
    list_facts[2].push_back(F2);
    list_facts[2].push_back(F3);


    procedural::Network N = procedural::Network(list_facts, "pick");
    N.displayNetwork();
    N.displayVariables();

    std::vector<procedural::Fact> facts;
    facts.emplace_back(true, "Bastien", "MoveThrought", "Cube");
    facts.emplace_back(true, "Bob", "hasInHand", "Cube");
    facts.emplace_back(true, "Bastien", "hasInHand", "Cube");
    facts.emplace_back(false, "Cube", "overSupport", "Table");

    for(auto& fact : facts)
    {
        std::cout << "--------------" << std::endl;
        N.evolve(fact);
        N.displayVariables();
        N.displayNetwork();
    }

    /*procedural::Fact Fp = procedural::Fact(true, "Bastien", "MoveThrought", "Cube");
    procedural::Fact Fp1 = procedural::Fact(true, "Bastien", "hasInHand", "Cube");
    procedural::Fact Fp2 = procedural::Fact(false, "Cube", "overSupport", "Table");
    Fact::individuals_table.printAll();
    std::cout << Fp.getProperty() << std::endl;
    N.evolve(Fp);
    N.displayVariables();
    N.displayNetwork();
    N.evolve(Fp1);
    N.displayVariables();
    N.displayNetwork();
    N.evolve(Fp2);
    N.displayVariables();
    N.displayNetwork();*/


    return 0;
}

