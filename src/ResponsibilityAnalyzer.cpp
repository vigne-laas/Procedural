#include "procedural/ResponsibilityAnalyzer.h"
#include <algorithm>
#include <map>
#include <sstream>

namespace procedural {

AttributionResult ResponsibilityAnalyzer::analyzeForClause(
    const std::string& for_clause,
    const std::vector<std::string>& action_parameters
) const {
    AttributionResult result;

    // Empty or missing FOR clause → UNCLEAR with low confidence
    if (for_clause.empty()) {
        result.attribution = "UNCLEAR";
        result.confidence = 0.3;
        result.failure_details = "No FOR clause specified - cannot determine responsibility";
        return result;
    }

    std::string normalized = normalizeIdentifier(for_clause);

    // Rule 1: FOR robot → SELF
    if (normalized == "robot" || normalized == "self") {
        result.attribution = "SELF";
        result.confidence = 0.9;
        result.failure_details = "Robot condition failed (FOR " + for_clause + ")";
        return result;
    }

    // Rule 2: FOR ?X (parameter) → PARTNER
    if (isParameter(for_clause, action_parameters)) {
        result.attribution = "PARTNER";
        result.confidence = 0.9;
        result.failure_details = "Partner condition failed (FOR " + for_clause + ")";
        return result;
    }

    // Rule 3: FOR environment → ENVIRONMENT
    if (normalized == "environment" || normalized == "world") {
        result.attribution = "ENVIRONMENT";
        result.confidence = 0.9;
        result.failure_details = "Environmental condition failed (FOR " + for_clause + ")";
        return result;
    }

    // Rule 4: FOR both(...) or unrecognized → UNCLEAR
    if (normalized.find("both") != std::string::npos) {
        result.attribution = "UNCLEAR";
        result.confidence = 0.5;
        result.failure_details = "Mixed responsibility (FOR " + for_clause + ")";
        return result;
    }

    // Default: unrecognized FOR clause
    result.attribution = "UNCLEAR";
    result.confidence = 0.4;
    result.failure_details = "Unrecognized FOR clause: " + for_clause;
    return result;
}

AttributionResult ResponsibilityAnalyzer::analyzeMultipleConditions(
    const std::vector<std::string>& for_clauses,
    const std::vector<std::string>& action_parameters
) const {
    // Handle empty case
    if (for_clauses.empty()) {
        AttributionResult result;
        result.attribution = "UNCLEAR";
        result.confidence = 0.3;
        result.failure_details = "No conditions provided for analysis";
        return result;
    }

    // Single condition - use direct analysis
    if (for_clauses.size() == 1) {
        return analyzeForClause(for_clauses[0], action_parameters);
    }

    // Multiple conditions - aggregate by priority
    std::map<std::string, int> attribution_counts;
    std::map<std::string, float> attribution_confidence_sum;
    std::vector<std::string> details_parts;

    // Analyze each condition
    for (const auto& clause : for_clauses) {
        auto analysis = analyzeForClause(clause, action_parameters);
        attribution_counts[analysis.attribution]++;
        attribution_confidence_sum[analysis.attribution] += analysis.confidence;
        details_parts.push_back(analysis.failure_details);
    }

    // Find highest priority attribution
    std::string highest_priority_attribution = "UNCLEAR";
    int highest_priority_value = 999;  // Lower is higher priority

    for (const auto& pair : attribution_counts) {
        int priority = getAttributionPriority(pair.first);
        if (priority < highest_priority_value) {
            highest_priority_value = priority;
            highest_priority_attribution = pair.first;
        }
    }

    // Build result
    AttributionResult result;
    result.attribution = highest_priority_attribution;

    // Calculate average confidence for the selected attribution
    int count = attribution_counts[highest_priority_attribution];
    float avg_confidence = attribution_confidence_sum[highest_priority_attribution] / count;

    // Reduce confidence slightly for mixed attributions
    if (attribution_counts.size() > 1) {
        avg_confidence *= 0.85;  // 15% penalty for mixed signals
    }

    result.confidence = std::min(0.95f, avg_confidence);  // Cap at 0.95

    // Build combined details
    std::ostringstream details;
    details << "Multiple conditions failed (" << for_clauses.size() << " total). ";
    details << "Primary responsibility: " << highest_priority_attribution;
    if (attribution_counts.size() > 1) {
        details << " (mixed attributions detected)";
    }
    result.failure_details = details.str();

    return result;
}

bool ResponsibilityAnalyzer::isParameter(
    const std::string& for_clause,
    const std::vector<std::string>& action_parameters
) const {
    // Empty clause is not a parameter
    if (for_clause.empty()) {
        return false;
    }

    // Check if starts with '?' (common parameter prefix)
    if (for_clause[0] == '?') {
        // If we have parameters list, verify it's actually in the list
        if (!action_parameters.empty()) {
            std::string normalized_clause = normalizeIdentifier(for_clause);
            for (const auto& param : action_parameters) {
                std::string normalized_param = normalizeIdentifier(param);
                if (normalized_clause == normalized_param) {
                    return true;
                }
            }
            return false;  // Has '?' but not in parameter list
        }
        return true;  // Has '?' and no parameter list to verify against
    }

    // Check if it matches any parameter name (without '?')
    if (!action_parameters.empty()) {
        std::string normalized_clause = normalizeIdentifier(for_clause);
        for (const auto& param : action_parameters) {
            std::string normalized_param = normalizeIdentifier(param);
            if (normalized_clause == normalized_param) {
                return true;
            }
        }
    }

    return false;
}

std::string ResponsibilityAnalyzer::normalizeIdentifier(const std::string& identifier) const {
    if (identifier.empty()) {
        return "";
    }

    std::string result = identifier;

    // Remove leading '?' if present
    if (result[0] == '?') {
        result = result.substr(1);
    }

    // Convert to lowercase for case-insensitive comparison
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);

    // Trim whitespace
    size_t start = result.find_first_not_of(" \t\n\r");
    size_t end = result.find_last_not_of(" \t\n\r");

    if (start != std::string::npos && end != std::string::npos) {
        result = result.substr(start, end - start + 1);
    }

    return result;
}

int ResponsibilityAnalyzer::getAttributionPriority(const std::string& attribution) const {
    // Priority order (lower value = higher priority):
    // 1. SELF - Robot's own faults have highest priority (safety-critical)
    // 2. PARTNER - Partner issues second priority
    // 3. ENVIRONMENT - External factors
    // 4. UNCLEAR - Fallback

    if (attribution == "SELF") return 0;
    if (attribution == "PARTNER") return 1;
    if (attribution == "ENVIRONMENT") return 2;
    if (attribution == "UNCLEAR") return 3;

    return 999;  // Unknown attribution type
}

} // namespace procedural
