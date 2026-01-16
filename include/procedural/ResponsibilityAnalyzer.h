#ifndef PROCEDURAL_RESPONSIBILITY_ANALYZER_H
#define PROCEDURAL_RESPONSIBILITY_ANALYZER_H

#include <string>
#include <vector>

namespace procedural {

/**
 * @brief Result of responsibility attribution analysis
 *
 * Simple structure following KISS principle with 4 possible attributions:
 * - SELF: Robot is responsible (FOR robot)
 * - PARTNER: Human/agent partner is responsible (FOR ?Param)
 * - ENVIRONMENT: External conditions are responsible (FOR environment)
 * - UNCLEAR: Mixed or undefined responsibility
 */
struct AttributionResult {
    std::string attribution;        // "SELF", "PARTNER", "ENVIRONMENT", "UNCLEAR"
    float confidence;               // 0.0 to 1.0
    std::string failure_details;    // Human-readable explanation
};

/**
 * @brief Analyzes commitment violations to determine responsibility attribution
 *
 * KISS Implementation - Simple rule-based attribution:
 *
 * Rule 1: FOR robot → SELF (confidence: 0.9)
 * Rule 2: FOR ?X (parameter) → PARTNER (confidence: 0.9)
 * Rule 3: FOR environment → ENVIRONMENT (confidence: 0.9)
 * Rule 4: FOR both(...) or missing FOR → UNCLEAR (confidence: 0.5/0.3)
 *
 * This analyzer provides automatic responsibility attribution without requiring
 * complex analysis or domain-specific knowledge.
 */
class ResponsibilityAnalyzer {
public:
    ResponsibilityAnalyzer() = default;
    ~ResponsibilityAnalyzer() = default;

    /**
     * @brief Analyze a single FOR clause to determine responsibility
     *
     * @param for_clause The FOR clause from commitment condition (e.g., "robot", "?C", "environment")
     * @param action_parameters List of action parameters to check if FOR references a parameter
     * @return AttributionResult with attribution, confidence, and details
     */
    AttributionResult analyzeForClause(
        const std::string& for_clause,
        const std::vector<std::string>& action_parameters = {}
    ) const;

    /**
     * @brief Analyze multiple violated conditions to determine overall responsibility
     *
     * When multiple conditions fail simultaneously, this method aggregates
     * the individual attributions to provide an overall result.
     *
     * Priority order (highest to lowest):
     * 1. SELF (robot's fault has priority for safety)
     * 2. PARTNER (partner issues second)
     * 3. ENVIRONMENT (external issues)
     * 4. UNCLEAR (fallback)
     *
     * @param for_clauses Vector of FOR clauses from all violated conditions
     * @param action_parameters List of action parameters
     * @return AttributionResult for overall failure
     */
    AttributionResult analyzeMultipleConditions(
        const std::vector<std::string>& for_clauses,
        const std::vector<std::string>& action_parameters = {}
    ) const;

private:
    /**
     * @brief Check if a FOR clause references an action parameter
     *
     * Parameters are typically prefixed with '?' (e.g., "?C", "?Client")
     *
     * @param for_clause The FOR clause to check
     * @param action_parameters List of action parameters (may include '?' prefix)
     * @return true if FOR clause references a parameter
     */
    bool isParameter(
        const std::string& for_clause,
        const std::vector<std::string>& action_parameters
    ) const;

    /**
     * @brief Extract base name from potentially prefixed identifier
     *
     * Handles cases like "?C" → "C", "robot" → "robot"
     *
     * @param identifier The identifier to normalize
     * @return Base name without prefix
     */
    std::string normalizeIdentifier(const std::string& identifier) const;

    /**
     * @brief Determine priority ranking for attribution types
     *
     * Used when aggregating multiple attributions.
     * Lower value = higher priority.
     *
     * @param attribution The attribution type
     * @return Priority value (0-3)
     */
    int getAttributionPriority(const std::string& attribution) const;
};

} // namespace procedural

#endif // PROCEDURAL_RESPONSIBILITY_ANALYZER_H
