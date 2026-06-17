#pragma once

#include "pmg/ParameterVector.h"

#include <random>
#include <vector>

namespace pmg {

// Simplex support for one parametric motion space.
//
// Vertices are parameter vectors in R^D. A full support has D + 1 vertices:
// two endpoints for 1-D, a triangle for 2-D, a tetrahedron for 3-D, etc.
// Barycentric weights are returned in vertex order and sum to one for points
// in the affine hull; negative weights mean the point is outside the simplex.
class ParameterSupport {
public:
    explicit ParameterSupport(std::vector<ParameterVector> vertices);

    int Dimension() const;
    int NumVertices() const;
    const std::vector<ParameterVector>& Vertices() const;

    std::vector<float> BarycentricWeights(const ParameterVector& parameter) const;
    bool Contains(const ParameterVector& parameter, float tolerance = 1.0e-5f) const;
    ParameterVector Project(const ParameterVector& parameter) const;
    ParameterVector SampleUniform(std::mt19937& rng) const;

private:
    std::vector<ParameterVector> vertices_;
};

}  // namespace pmg
