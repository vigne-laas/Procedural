#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "procedural/structures/Fact.h"
#include <ontologenius/OntologyManipulator.h>
#include "procedural/utils/Logger.h"
#include<unistd.h> // pour Linux


// Mock class for ObjectPropertyClient
onto::OntologyManipulator* mock_;

// Test suite for Fact
class FactTest : public ::testing::Test {
protected:
    FactTest()
    {
        procedural::WordTable::properties_table.get("property");
        procedural::WordTable::properties_table.get("property1");
        procedural::WordTable::properties_table.get("property2");
        // You can do set-up work for each test here.
    }

    ~FactTest() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }
};

// Test suite for Fact
class FactTestOntology : public ::testing::Test {
protected:
    FactTestOntology()
    {
        procedural::WordTable::properties_table.get("property");
        procedural::WordTable::properties_table.get("property1");
        procedural::WordTable::properties_table.get("property2");

//        LOG_INFO << "waiting for connection to ontology feeder";
//        mock_->feeder.waitConnected();
//        mock_->feeder.addProperty("myself", "property", "myself");
//        LOG_INFO << "before waitUpdate 500";
//        mock_->feeder.waitUpdate(1000);
//        mock_->feeder.addInheritage("property1", "property");
//        mock_->feeder.addInheritage("property2", "property1");
//        LOG_INFO << "before waitUpdate 500";
//        mock_->feeder.waitUpdate(1000);

        // You can do set-up work for each test here.
    }

    ~FactTestOntology() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }
};

// Test case for identical facts
TEST_F(FactTest, testIdenticalFacts)
{
    procedural::Fact fact1(true, "subject", "type_subject", "property", "object", "type_object");
    procedural::Fact fact2(true, "subject", "type_subject", "property", "object", "type_object");
    ASSERT_TRUE(fact1.match(fact2));
}

// Test case for facts with different properties
TEST_F(FactTest, testDifferentProperties)
{
    procedural::Fact fact1(true, "subject", "type_subject", "property1", "object", "type_object",
                           procedural::TimeStamp_t());
    procedural::Fact fact2(true, "subject", "type_subject", "property2", "object", "type_object",
                           procedural::TimeStamp_t());
    ASSERT_FALSE(fact1.match(fact2));
}

// Test case for facts with different subjects
TEST_F(FactTest, testDifferentSubjects)
{
    procedural::Fact fact1(true, "subject1", "type_subject", "property", "object", "type_object",
                           procedural::TimeStamp_t());
    procedural::Fact fact2(true, "subject2", "type_subject", "property", "object", "type_object",
                           procedural::TimeStamp_t());
    ASSERT_FALSE(fact1.match(fact2));
}

// Test case for facts with different objects
TEST_F(FactTest, testDifferentObjects)
{
    procedural::Fact fact1(true, "subject", "type_subject", "property", "object1", "type_object",
                           procedural::TimeStamp_t());
    procedural::Fact fact2(true, "subject", "type_subject", "property", "object2", "type_object",
                           procedural::TimeStamp_t());
    ASSERT_FALSE(fact1.match(fact2));
}

// Test case for facts with different actions
TEST_F(FactTest, testDifferentActions)
{
    procedural::Fact fact1(true, "subject", "type_subject", "property", "object", "type_object",
                           procedural::TimeStamp_t());
    procedural::Fact fact2(false, "subject", "type_subject", "property", "object", "type_object",
                           procedural::TimeStamp_t());
    ASSERT_FALSE(fact1.match(fact2));
}

// TODO : test case with different types in subject and/or object
TEST_F(FactTest, testDifferentTypes)
{
    procedural::Fact fact1(true, "subject", "type_subject", "property", "object", "type_object");
    procedural::Fact fact2(true, "subject", "type_subject2", "property", "object", "type_object");
    ASSERT_FALSE(fact1.match(fact2));
}

TEST_F(FactTest, testDifferentTypes2)
{
    procedural::Fact fact1(true, "subject", "type_subject", "property", "object", "type_object");
    procedural::Fact fact2(true, "subject", "type_subject", "property", "object", "type_object2");
    ASSERT_FALSE(fact1.match(fact2));
}

// // Test case for expandProperty and match
TEST(FactTestOntology, testExpandPropertyAndMatch)
{
    // Create a mock object for ObjectPropertyClient
    //onto::OntologyManipulator mock;
    // Create an instance of Fact and call expandProperty
    LOG_INFO << "waiting for connection to ontology feeder";
    mock_->feeder.waitConnected();
    mock_->feeder.addProperty("myself", "property", "myself");
    LOG_INFO << "before waitUpdate 500";
    mock_->feeder.waitUpdate(1000);
    mock_->feeder.addInheritage("property1", "property");
    mock_->feeder.addInheritage("property2", "property1");
    LOG_INFO << "before waitUpdate 500";
    mock_->feeder.waitUpdate(1000);
//    LOG_DEBUG << procedural::WordTable::properties_table.toString();
    procedural::Fact fact1(true, "subject", "type_subject", "property", "object", "type_object",
                           procedural::TimeStamp_t());
//    LOG_INFO << "before expandProperty : property id " << fact1.getIdProperty();
    fact1.expandProperty(&mock_->objectProperties);
    procedural::Fact fact2(true, "subject", "type_subject", "property1", "object", "type_object",
                           procedural::TimeStamp_t());
    fact2.expandProperty(&mock_->objectProperties);
    LOG_INFO << "fact1 : " << fact1.toString();
    LOG_INFO << "fact2 : " << fact2.toString();
    ASSERT_TRUE(fact1.match(fact2));
}

TEST(FactTestOntology, testExpandPropertyOnlyOneandMatch)
{
    LOG_INFO << "testExpand Property Only One and Match";
    // Create a mock object for ObjectPropertyClient
    //onto::OntologyManipulator mock;
    // Create an instance of Fact and call expandProperty
    LOG_INFO << "waiting for connection to ontology feeder";
    mock_->feeder.waitConnected();
    mock_->feeder.addProperty("myself", "property", "myself");
    LOG_INFO << "before waitUpdate 500";
    mock_->feeder.waitUpdate(1000);
    mock_->feeder.addInheritage("property1", "property");
    mock_->feeder.addInheritage("property2", "property1");
    LOG_INFO << "before waitUpdate 500";
    mock_->feeder.waitUpdate(1000);
    procedural::Fact fact1(true, "subject", "type_subject","property", "object", "type_object",procedural::TimeStamp_t());
    fact1.expandProperty(&mock_->objectProperties);
    procedural::Fact fact2(true, "subject", "type_subject","property1", "object","type_object", procedural::TimeStamp_t());
    ASSERT_TRUE(fact1.match(fact2));

    procedural::WordTable::properties_table.get("property3");
    procedural::Fact fact3(true, "subject","type_subject", "property4", "object","type_object", procedural::TimeStamp_t());
    ASSERT_FALSE(fact1.match(fact3));

}


// Run all the tests
int main(int argc, char** argv)
{
    Logger::setMinLevel(Logger::DEBUG);
    ros::init(argc, argv, "test_Fact");

    onto::OntologyManipulator mock;
    mock.close();
    LOG_INFO << "sleeping 2 seconds after close ontology";
    sleep(2);

    mock_ = &mock;

    //ros::start();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}