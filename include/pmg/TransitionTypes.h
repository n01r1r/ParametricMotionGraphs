#pragma once

#include "pmg/ParameterVector.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace pmg {

struct ParameterAabb {
    ParameterVector min_corner;
    ParameterVector max_corner;

    bool IsValid() const {
        if (min_corner.size() != max_corner.size()) {
            return false;
        }

        for (std::size_t index = 0; index < min_corner.size(); ++index) {
            if (min_corner[index] > max_corner[index]) {
                return false;
            }
        }
        return true;
    }

    // True when the box bounds no region: never populated, mismatched corners,
    // or some axis with min > max (e.g. the Empty() factory before any
    // ExpandToInclude, or a box collapsed past-empty by ShrinkToExclude).
    bool IsEmpty() const {
        if (min_corner.empty() || max_corner.empty() ||
            min_corner.size() != max_corner.size()) {
            return true;
        }
        for (std::size_t index = 0; index < min_corner.size(); ++index) {
            if (min_corner[index] > max_corner[index]) {
                return true;
            }
        }
        return false;
    }

    bool Contains(const ParameterVector& parameter) const {
        RequireSameParameterDimension(min_corner, parameter, "ParameterAabb::Contains");
        RequireSameParameterDimension(max_corner, parameter, "ParameterAabb::Contains");

        for (std::size_t index = 0; index < parameter.size(); ++index) {
            if (parameter[index] < min_corner[index] || parameter[index] > max_corner[index]) {
                return false;
            }
        }
        return true;
    }

    ParameterVector Clamp(const ParameterVector& parameter) const {
        RequireSameParameterDimension(min_corner, parameter, "ParameterAabb::Clamp");
        RequireSameParameterDimension(max_corner, parameter, "ParameterAabb::Clamp");

        ParameterVector clamped = parameter;
        for (std::size_t index = 0; index < clamped.size(); ++index) {
            clamped[index] = std::clamp(clamped[index], min_corner[index], max_corner[index]);
        }
        return clamped;
    }

    static ParameterAabb Empty(int dimension) {
        ParameterAabb box;
        box.min_corner.assign(dimension, std::numeric_limits<float>::infinity());
        box.max_corner.assign(dimension, -std::numeric_limits<float>::infinity());
        return box;
    }

    void ExpandToInclude(const ParameterVector& parameter) {
        if (min_corner.empty()) {
            min_corner = parameter;
            max_corner = parameter;
            return;
        }

        RequireSameParameterDimension(min_corner, parameter, "ParameterAabb::ExpandToInclude");
        RequireSameParameterDimension(max_corner, parameter, "ParameterAabb::ExpandToInclude");

        for (std::size_t index = 0; index < parameter.size(); ++index) {
            min_corner[index] = std::min(min_corner[index], parameter[index]);
            max_corner[index] = std::max(max_corner[index], parameter[index]);
        }
    }

    // Minimally shrink the box so `point` falls just outside it (PMG paper §3.2,
    // Fig. 4c). Picks the single dimension/face whose inward distance is
    // smallest (least volume lost) and pulls that face `epsilon` past the point.
    // If the point cannot be excluded while preserving a non-empty box, the
    // chosen face is allowed to cross the opposite face; the caller must handle
    // the resulting empty box.
    void ShrinkToExclude(const ParameterVector& point, float epsilon) {
        if (!Contains(point)) {
            return;
        }

        std::size_t best_dimension = 0;
        bool cut_min_face = true;  // true: raise min above point; false: lower max
        float best_inward_distance = std::numeric_limits<float>::infinity();
        for (std::size_t index = 0; index < point.size(); ++index) {
            const float distance_to_min_face = point[index] - min_corner[index];
            const float distance_to_max_face = max_corner[index] - point[index];
            if (distance_to_min_face <= distance_to_max_face) {
                if (distance_to_min_face < best_inward_distance) {
                    best_inward_distance = distance_to_min_face;
                    best_dimension = index;
                    cut_min_face = true;
                }
            } else if (distance_to_max_face < best_inward_distance) {
                best_inward_distance = distance_to_max_face;
                best_dimension = index;
                cut_min_face = false;
            }
        }

        if (cut_min_face) {
            min_corner[best_dimension] = point[best_dimension] + epsilon;
        } else {
            max_corner[best_dimension] = point[best_dimension] - epsilon;
        }
    }
};

// Directional phase pair measured at one retained GOOD target parameter.
// Source phase identifies the first source support frame; target phase
// identifies the last target support frame. Runtime evaluates these samples at
// the requested target instead of using one in-box average.
struct TargetTransitionPhaseSample {
    ParameterVector target_parameter;
    float source_transition_phase = 0.85f;
    float target_transition_phase = 0.15f;
};

// One sampled source parameter, its reachable target box, and target-dependent
// directional transition phases. Scalar phases are V2-V5 fallback/summary
// values with the same first-source/last-target meaning.
struct TransitionSample {
    ParameterVector source_parameter;
    ParameterAabb target_parameter_box;
    float source_transition_phase = 0.85f;
    float target_transition_phase = 0.15f;
    // Build-time transition quality: mean Kovar metric distance D over the GOOD
    // target samples retained inside target_parameter_box (paper §3.1/§3.2). It
    // is the offline D the runtime carries forward so a streamed transition can
    // be tagged with the edge quality it was scheduled from. V9+ (zero when
    // absent in older artifacts).
    float transition_distance = 0.0f;
    std::vector<TargetTransitionPhaseSample> target_phase_samples;
};

}  // namespace pmg
