// test_ObservationFact.cpp
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "procedural/structures/Observation.h"
#include "procedural/structures/ObservationFact.h"
#include "procedural/structures/Fact.h"
#include "procedural/utils/Logger.h"
#include "procedural/structures/VariablesTable.h"


TEST(ObservationFactTest, testConstructor)
{
    procedural::Fact fact(true, "subject", "property", "object", procedural::TimeStamp_t());
    procedural::ObservationFact obsFact(fact);

    ASSERT_EQ(obsFact.getId(), ((int64_t) 0x01 << 63) | (int64_t) std::hash<std::string>{}(
            (fact.getAdd() ? "ADD " : "DEL ") + fact.getProperty()));
}

TEST(ObservationFactTest, testEqualityOperator)
{
    procedural::Fact::properties_table.get("property1");
    procedural::Fact::properties_table.get("property2");

    procedural::Fact fact1(true, "subject1", "property1", "object1", procedural::TimeStamp_t());
    procedural::Fact fact2(false, "subject2", "property2", "object2", procedural::TimeStamp_t());
    procedural::ObservationFact obsFact1(fact1);
    procedural::ObservationFact obsFact2(fact2);
    LOG_INFO << "Test de l'opérateur d'égalité";
    ASSERT_TRUE(obsFact1 == obsFact1);
    LOG_INFO << "Test de l'opérateur d'inégalité";
    ASSERT_FALSE(obsFact1 == obsFact2);
}

TEST(ObservationTest, testConstructor)
{
    procedural::Observation obs(1);
    ASSERT_EQ(obs.getId(), 1);
}

TEST(ObservationTest, testEqualityOperator)
{
    procedural::Observation obs1(1);
    procedural::Observation obs2(2);
    ASSERT_TRUE(obs1 == obs1);
    ASSERT_FALSE(obs1 == obs2);
}

TEST(Observation_FactTest, testWithFact)
{
    procedural::Fact fact(true, "subject", "property", "object", procedural::TimeStamp_t());
    procedural::ObservationFact obs(fact);
    procedural::Observation obs2(1);
    ASSERT_FALSE(obs == obs2);
}

TEST(Observation_FactTest, testWithObservationFact)
{
    procedural::Fact::properties_table.get("property");
    procedural::Fact fact(true, "subject", "property", "object", procedural::TimeStamp_t());
    procedural::Fact fact2(true, "", "property", "", procedural::TimeStamp_t());
    procedural::Fact fact3(true, "subject", "property", "", procedural::TimeStamp_t());
    procedural::Fact fact4(true, "", "property", "object", procedural::TimeStamp_t());
    procedural::Fact fact10(false, "", "property", "", procedural::TimeStamp_t());
    procedural::Fact fact5(true, "sub", "property", "", procedural::TimeStamp_t());
    procedural::Fact fact6(true, "subject", "property", "obj", procedural::TimeStamp_t());
    procedural::Fact fact7(true, "sub", "property", "object", procedural::TimeStamp_t());

    procedural::ObservationFact obs(fact);
    procedural::ObservationFact obs2(fact2);
    procedural::ObservationFact obs3(fact3);
    procedural::ObservationFact obs4(fact4);
    procedural::ObservationFact obs5(fact5);
    procedural::ObservationFact obs6(fact6);
    procedural::ObservationFact obs7(fact7);
    procedural::ObservationFact obs10(fact10);
    LOG_INFO << "Test des cas avec faits incomplets mais juste \n";
    ASSERT_TRUE(obs == obs2);
    ASSERT_TRUE(obs == obs3);
    ASSERT_TRUE(obs == obs4);
    LOG_INFO << "Test des cas avec faits partiels mais faux \n";
    ASSERT_FALSE(obs == obs5);
    ASSERT_FALSE(obs == obs6);
    ASSERT_FALSE(obs == obs7);
    ASSERT_FALSE(obs == obs10);
}

TEST(Observation_FactTest, testWithVectorObservation)
{
    procedural::Fact fact(true, "subject", "property", "object", procedural::TimeStamp_t());
    procedural::ObservationFact obs(fact);
    procedural::Observation obs2(1);
    procedural::Fact fact2(false, "subject", "property", "object", procedural::TimeStamp_t());
    procedural::ObservationFact obs3(fact2);
    std::vector<procedural::Observation*> vecObs;
    vecObs.push_back(&obs);
    vecObs.push_back(&obs2);
    for (auto& o: vecObs) {
        LOG_INFO << "Observation : " << o->getId() << (o->getId() < 0 ? " is a fact" : " is an observation");
        ASSERT_FALSE(*o == obs3);
    }

}


TEST(ObservationTest, testLinkVariables)
{


    // Créer des variables à lier
    std::shared_ptr<procedural::Variable_t> var1 = std::make_shared<procedural::Variable_t>("A");
    std::shared_ptr<procedural::Variable_t> var2 = std::make_shared<procedural::Variable_t>("B");
    std::shared_ptr<procedural::Variable_t> var3 = std::make_shared<procedural::Variable_t>("C");

    std::shared_ptr<procedural::Variable_t> var4 = std::make_shared<procedural::Variable_t>("A");
    var4->setValue(1);
    std::shared_ptr<procedural::Variable_t> var5 = std::make_shared<procedural::Variable_t>("B");
    var5->setValue(2);
    std::shared_ptr<procedural::Variable_t> var6 = std::make_shared<procedural::Variable_t>("C");
    var6->setValue(3);

    // Créer une map de variables
    std::map<std::string, std::shared_ptr<procedural::Variable_t>> variables_unset;
    variables_unset["A"] = var1;
    variables_unset["B"] = var2;
    variables_unset["C"] = var3;

    std::map<std::string, std::shared_ptr<procedural::Variable_t>> variables_set;
    variables_set["A"] = var4;
    variables_set["B"] = var5;
    variables_set["C"] = var6;

    // Créer une observation
    procedural::VariableTable_t table;
    table.variables = variables_unset;
    table.agents.insert("agent1");
    procedural::Observation obs(1, table);

    std::pair<int64_t, procedural::VariableTable_t> data = obs.getData();
    LOG_INFO << "Test avant link des variables";
//    LOG_DEBUG << "Variables de l'observation : ";
//    for (auto& var: data.second.variables) {
//        LOG_DEBUG << var.first << " : " << var.second->getValue();
//    }

    ASSERT_EQ(data.second.variables["A"]->getValue(), 0);
    ASSERT_EQ(data.second.variables["B"]->getValue(), 0);
    ASSERT_EQ(data.second.variables["C"]->getValue(), 0);

    // Lier les variables à l'observation
    obs.linkVariables(variables_set);
    LOG_INFO << "Test après link des variables";
    data = obs.getData();
//    LOG_DEBUG << "Variables de l'observation : ";
//    for (auto& var: data.second.variables) {
//        LOG_DEBUG << var.first << " : " << var.second->getValue();
//    }

    ASSERT_EQ(data.second.variables["A"], var4);
    ASSERT_EQ(data.second.variables["A"]->getValue(), 1);
    ASSERT_EQ(data.second.variables["B"], var5);
    ASSERT_EQ(data.second.variables["B"]->getValue(), 2);
    ASSERT_EQ(data.second.variables["C"], var6);
    ASSERT_EQ(data.second.variables["C"]->getValue(), 3);
}

int main(int argc, char** argv)
{
    SET_MIN_LEVEL(Logger::DEBUG);
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_ObservationFact");
    return RUN_ALL_TESTS();
}