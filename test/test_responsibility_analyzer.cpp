#include <gtest/gtest.h>
#include "procedural/ResponsibilityAnalyzer.h"

namespace procedural {

TEST(ResponsibilityAnalyzerTest, testForRobot) {
    ResponsibilityAnalyzer analyzer;

    auto result = analyzer.analyzeForClause("robot");

    EXPECT_EQ(result.attribution, "SELF");
    EXPECT_FLOAT_EQ(result.confidence, 0.9f);
    EXPECT_FALSE(result.failure_details.empty());
    std::cout << "FOR robot → " << result.attribution
              << " (confidence: " << result.confidence << ")" << std::endl;
}

TEST(ResponsibilityAnalyzerTest, testForSelf) {
    ResponsibilityAnalyzer analyzer;

    auto result = analyzer.analyzeForClause("self");

    EXPECT_EQ(result.attribution, "SELF");
    EXPECT_FLOAT_EQ(result.confidence, 0.9f);
}

TEST(ResponsibilityAnalyzerTest, testForParameter) {
    ResponsibilityAnalyzer analyzer;
    std::vector<std::string> params = {"Client", "Area"};

    // Test with '?' prefix
    auto result1 = analyzer.analyzeForClause("?Client", params);
    EXPECT_EQ(result1.attribution, "PARTNER");
    EXPECT_FLOAT_EQ(result1.confidence, 0.9f);
    std::cout << "FOR ?Client → " << result1.attribution
              << " (confidence: " << result1.confidence << ")" << std::endl;

    // Test without '?' prefix but in parameter list
    auto result2 = analyzer.analyzeForClause("Client", params);
    EXPECT_EQ(result2.attribution, "PARTNER");
    EXPECT_FLOAT_EQ(result2.confidence, 0.9f);

    // Test with '?C' (short form)
    std::vector<std::string> params2 = {"C"};
    auto result3 = analyzer.analyzeForClause("?C", params2);
    EXPECT_EQ(result3.attribution, "PARTNER");
    EXPECT_FLOAT_EQ(result3.confidence, 0.9f);
    std::cout << "FOR ?C → " << result3.attribution
              << " (confidence: " << result3.confidence << ")" << std::endl;
}

TEST(ResponsibilityAnalyzerTest, testForEnvironment) {
    ResponsibilityAnalyzer analyzer;

    auto result1 = analyzer.analyzeForClause("environment");
    EXPECT_EQ(result1.attribution, "ENVIRONMENT");
    EXPECT_FLOAT_EQ(result1.confidence, 0.9f);
    std::cout << "FOR environment → " << result1.attribution
              << " (confidence: " << result1.confidence << ")" << std::endl;

    auto result2 = analyzer.analyzeForClause("world");
    EXPECT_EQ(result2.attribution, "ENVIRONMENT");
    EXPECT_FLOAT_EQ(result2.confidence, 0.9f);
}

TEST(ResponsibilityAnalyzerTest, testForBoth) {
    ResponsibilityAnalyzer analyzer;

    auto result = analyzer.analyzeForClause("both(robot,?C)");
    EXPECT_EQ(result.attribution, "UNCLEAR");
    EXPECT_FLOAT_EQ(result.confidence, 0.5f);
    std::cout << "FOR both(...) → " << result.attribution
              << " (confidence: " << result.confidence << ")" << std::endl;
}

TEST(ResponsibilityAnalyzerTest, testEmptyForClause) {
    ResponsibilityAnalyzer analyzer;

    auto result = analyzer.analyzeForClause("");
    EXPECT_EQ(result.attribution, "UNCLEAR");
    EXPECT_FLOAT_EQ(result.confidence, 0.3f);
    std::cout << "FOR (empty) → " << result.attribution
              << " (confidence: " << result.confidence << ")" << std::endl;
}

TEST(ResponsibilityAnalyzerTest, testUnrecognizedForClause) {
    ResponsibilityAnalyzer analyzer;

    auto result = analyzer.analyzeForClause("unknown_identifier");
    EXPECT_EQ(result.attribution, "UNCLEAR");
    EXPECT_LE(result.confidence, 0.5f);
}

TEST(ResponsibilityAnalyzerTest, testMultipleConditionsSamePriority) {
    ResponsibilityAnalyzer analyzer;
    std::vector<std::string> for_clauses = {"robot", "robot"};

    auto result = analyzer.analyzeMultipleConditions(for_clauses);
    EXPECT_EQ(result.attribution, "SELF");
    std::cout << "Multiple SELF → " << result.attribution
              << " (confidence: " << result.confidence << ")" << std::endl;
}

TEST(ResponsibilityAnalyzerTest, testMultipleConditionsMixedPriority) {
    ResponsibilityAnalyzer analyzer;
    std::vector<std::string> params = {"C"};

    // SELF has highest priority
    std::vector<std::string> for_clauses = {"robot", "environment", "?C"};

    auto result = analyzer.analyzeMultipleConditions(for_clauses, params);
    EXPECT_EQ(result.attribution, "SELF");
    EXPECT_LT(result.confidence, 0.9f);  // Reduced due to mixed signals
    std::cout << "Mixed (SELF + ENVIRONMENT + PARTNER) → " << result.attribution
              << " (confidence: " << result.confidence << ")" << std::endl;
}

TEST(ResponsibilityAnalyzerTest, testSingleCondition) {
    ResponsibilityAnalyzer analyzer;
    std::vector<std::string> for_clauses = {"robot"};

    auto result = analyzer.analyzeMultipleConditions(for_clauses);
    EXPECT_EQ(result.attribution, "SELF");
    EXPECT_FLOAT_EQ(result.confidence, 0.9f);
}

TEST(ResponsibilityAnalyzerTest, testEmptyConditionsList) {
    ResponsibilityAnalyzer analyzer;
    std::vector<std::string> for_clauses;

    auto result = analyzer.analyzeMultipleConditions(for_clauses);
    EXPECT_EQ(result.attribution, "UNCLEAR");
    EXPECT_FLOAT_EQ(result.confidence, 0.3f);
}

} // namespace procedural

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
