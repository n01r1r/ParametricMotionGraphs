#pragma once

#include "pmg/ParameterVector.h"
#include "pmg/TransitionTypes.h"

#include <algorithm>
#include <random>
#include <stdexcept>
#include <vector>

namespace pmg {

// Continuous parameter domain used by PMG nodes. Heck & Gleicher assume each
// node represents a smooth parametric motion space; this class makes the valid
// sampling/clamping region explicit instead of scattering min/max logic through
// the builder/runtime.
class ParameterDomain {
public:
    ParameterDomain() = default;

    explicit ParameterDomain(int dimension)
        : bounds_(ParameterAabb::Empty(dimension)) {
        if (dimension <= 0) {
            throw std::runtime_error("ParameterDomain: dimension must be positive");
        }
    }

    explicit ParameterDomain(ParameterAabb bounds)
        : bounds_(std::move(bounds)) {
        if (bounds_.IsEmpty()) {
            throw std::runtime_error("ParameterDomain: bounds must not be empty");
        }
    }

    int Dimension() const {
        return static_cast<int>(bounds_.min_corner.size());
    }

    const ParameterAabb& Bounds() const {
        return bounds_;
    }

    bool IsEmpty() const {
        return bounds_.IsEmpty();
    }

    bool Contains(const ParameterVector& parameter) const {
        return !bounds_.IsEmpty() && bounds_.Contains(parameter);
    }

    ParameterVector Clamp(const ParameterVector& parameter) const {
        if (bounds_.IsEmpty()) {
            throw std::runtime_error("ParameterDomain::Clamp: empty domain");
        }
        return bounds_.Clamp(parameter);
    }

    void ExpandToInclude(const ParameterVector& parameter) {
        if (!bounds_.min_corner.empty()) {
            RequireSameParameterDimension(bounds_.min_corner, parameter,
                                          "ParameterDomain::ExpandToInclude");
        }
        bounds_.ExpandToInclude(parameter);
    }

    ParameterVector SampleUniform(std::mt19937& rng) const {
        if (bounds_.IsEmpty()) {
            throw std::runtime_error("ParameterDomain::SampleUniform: empty domain");
        }

        ParameterVector sample(bounds_.min_corner.size(), 0.0f);
        for (std::size_t dimension = 0; dimension < sample.size(); ++dimension) {
            std::uniform_real_distribution<float> distribution(
                bounds_.min_corner[dimension], bounds_.max_corner[dimension]);
            sample[dimension] = distribution(rng);
        }
        return sample;
    }

    static ParameterDomain FromExamples(const std::vector<ParameterVector>& parameters) {
        if (parameters.empty()) {
            throw std::runtime_error("ParameterDomain::FromExamples: no parameters");
        }

        ParameterDomain domain(static_cast<int>(parameters.front().size()));
        for (const ParameterVector& parameter : parameters) {
            domain.ExpandToInclude(parameter);
        }
        return domain;
    }

private:
    ParameterAabb bounds_;
};

}  // namespace pmg
