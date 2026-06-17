#include "pmg/ParameterSupport.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pmg {
namespace {

constexpr float kPivotTolerance = 1.0e-8f;
constexpr float kWeightTolerance = 1.0e-5f;
constexpr int kMaxEnumeratedSimplexVertices = 20;

bool IsFiniteVector(const ParameterVector& values) {
    for (const float value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

float Dot(const ParameterVector& left, const ParameterVector& right) {
    RequireSameParameterDimension(left, right, "ParameterSupport::Dot");
    float sum = 0.0f;
    for (std::size_t index = 0; index < left.size(); ++index) {
        sum += left[index] * right[index];
    }
    return sum;
}

ParameterVector Subtract(const ParameterVector& left, const ParameterVector& right) {
    RequireSameParameterDimension(left, right, "ParameterSupport::Subtract");
    ParameterVector difference(left.size(), 0.0f);
    for (std::size_t index = 0; index < left.size(); ++index) {
        difference[index] = left[index] - right[index];
    }
    return difference;
}

ParameterVector Combine(
    const std::vector<ParameterVector>& vertices,
    const std::vector<float>& weights) {
    if (vertices.empty()) {
        throw std::runtime_error("ParameterSupport::Combine: no vertices");
    }
    ParameterVector point(vertices.front().size(), 0.0f);
    for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
        for (std::size_t dimension = 0; dimension < point.size(); ++dimension) {
            point[dimension] += weights[vertex] * vertices[vertex][dimension];
        }
    }
    return point;
}

std::vector<float> SolveLinearSystem(
    std::vector<std::vector<float>> matrix,
    std::vector<float> rhs,
    const char* context) {
    const std::size_t size = rhs.size();
    for (std::size_t column = 0; column < size; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < size; ++row) {
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(matrix[pivot][column]) <= kPivotTolerance) {
            throw std::runtime_error(std::string(context) + ": degenerate simplex");
        }
        std::swap(matrix[column], matrix[pivot]);
        std::swap(rhs[column], rhs[pivot]);

        const float pivot_value = matrix[column][column];
        for (std::size_t row = column + 1; row < size; ++row) {
            const float factor = matrix[row][column] / pivot_value;
            for (std::size_t entry = column; entry < size; ++entry) {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            rhs[row] -= factor * rhs[column];
        }
    }

    std::vector<float> solution(size, 0.0f);
    for (std::size_t offset = 0; offset < size; ++offset) {
        const std::size_t row = size - 1 - offset;
        float value = rhs[row];
        for (std::size_t column = row + 1; column < size; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    return solution;
}

std::vector<float> BarycentricForVertices(
    const std::vector<ParameterVector>& vertices,
    const ParameterVector& parameter) {
    if (vertices.size() == 1) {
        return {1.0f};
    }

    const std::size_t coordinate_count = vertices.size() - 1;
    std::vector<ParameterVector> edges;
    edges.reserve(coordinate_count);
    for (std::size_t vertex = 1; vertex < vertices.size(); ++vertex) {
        edges.push_back(Subtract(vertices[vertex], vertices.front()));
    }
    const ParameterVector offset = Subtract(parameter, vertices.front());

    std::vector<std::vector<float>> gram(
        coordinate_count, std::vector<float>(coordinate_count, 0.0f));
    std::vector<float> rhs(coordinate_count, 0.0f);
    for (std::size_t row = 0; row < coordinate_count; ++row) {
        rhs[row] = Dot(edges[row], offset);
        for (std::size_t column = 0; column < coordinate_count; ++column) {
            gram[row][column] = Dot(edges[row], edges[column]);
        }
    }

    std::vector<float> coordinates =
        SolveLinearSystem(gram, rhs, "ParameterSupport::BarycentricWeights");
    std::vector<float> weights(vertices.size(), 0.0f);
    weights.front() = 1.0f;
    for (std::size_t coordinate = 0; coordinate < coordinates.size(); ++coordinate) {
        weights[coordinate + 1] = coordinates[coordinate];
        weights.front() -= coordinates[coordinate];
    }
    return weights;
}

}  // namespace

ParameterSupport::ParameterSupport(std::vector<ParameterVector> vertices)
    : type_(Type::kSimplex), vertices_(std::move(vertices)) {
    if (vertices_.empty()) {
        throw std::runtime_error("ParameterSupport: vertices must not be empty");
    }
    if (vertices_.front().empty()) {
        throw std::runtime_error("ParameterSupport: vertex dimension must be positive");
    }
    for (const ParameterVector& vertex : vertices_) {
        RequireSameParameterDimension(vertices_.front(), vertex, "ParameterSupport");
        if (!IsFiniteVector(vertex)) {
            throw std::runtime_error("ParameterSupport: vertices must be finite");
        }
    }
    if (vertices_.size() != vertices_.front().size() + 1) {
        throw std::runtime_error("ParameterSupport: simplex needs dimension + 1 vertices");
    }
    (void)BarycentricForVertices(vertices_, vertices_.front());
}

ParameterSupport ParameterSupport::CreateTriangulated2D(
    std::vector<ParameterVector> vertices,
    std::vector<std::array<int, 3>> triangles) {
    return ParameterSupport(std::move(vertices), std::move(triangles));
}

ParameterSupport::ParameterSupport(
    std::vector<ParameterVector> vertices,
    std::vector<std::array<int, 3>> triangles)
    : type_(Type::kTriangulated2D),
      vertices_(std::move(vertices)),
      triangles_(std::move(triangles)) {
    if (vertices_.empty() || vertices_.front().size() != 2) {
        throw std::runtime_error("ParameterSupport: triangulated 2D requires 2D vertices");
    }
    if (triangles_.empty()) {
        throw std::runtime_error("ParameterSupport: triangulated 2D requires triangles");
    }
}

ParameterSupport::Type ParameterSupport::GetType() const {
    return type_;
}

const std::vector<std::array<int, 3>>& ParameterSupport::Triangles() const {
    return triangles_;
}

int ParameterSupport::Dimension() const {
    return static_cast<int>(vertices_.front().size());
}

int ParameterSupport::NumVertices() const {
    return static_cast<int>(vertices_.size());
}

const std::vector<ParameterVector>& ParameterSupport::Vertices() const {
    return vertices_;
}

std::vector<float> ParameterSupport::BarycentricWeights(
    const ParameterVector& parameter) const {
    RequireSameParameterDimension(vertices_.front(), parameter,
                                  "ParameterSupport::BarycentricWeights");
    if (!IsFiniteVector(parameter)) {
        throw std::runtime_error(
            "ParameterSupport::BarycentricWeights: parameter must be finite");
    }
    if (type_ == Type::kTriangulated2D) {
        ParameterVector p = Project(parameter);
        int best_tri = -1;
        std::vector<float> best_tri_weights;
        float best_dist = std::numeric_limits<float>::infinity();

        for (std::size_t i = 0; i < triangles_.size(); ++i) {
            const auto& t = triangles_[i];
            std::vector<ParameterVector> tri_vertices = { vertices_[t[0]], vertices_[t[1]], vertices_[t[2]] };
            std::vector<float> w;
            try {
                w = BarycentricForVertices(tri_vertices, p);
            } catch (...) {
                continue;
            }

            bool inside = true;
            for (float weight : w) {
                if (weight < -kWeightTolerance) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                best_tri = static_cast<int>(i);
                best_tri_weights = std::move(w);
                break;
            }

            ParameterSupport tri(tri_vertices);
            ParameterVector p_tri = tri.Project(p);
            float dist = SquaredDistance(p, p_tri);
            if (dist < best_dist) {
                best_dist = dist;
                best_tri = static_cast<int>(i);
                best_tri_weights = std::move(w);
            }
        }

        if (best_tri == -1) {
            throw std::runtime_error("ParameterSupport::BarycentricWeights: no containing triangle found");
        }

        float sum = 0.0f;
        for (float& w : best_tri_weights) {
            w = std::max(0.0f, w);
            sum += w;
        }
        if (sum > 0.0f) {
            for (float& w : best_tri_weights) {
                w /= sum;
            }
        }

        std::vector<float> weights(vertices_.size(), 0.0f);
        const auto& t = triangles_[best_tri];
        weights[t[0]] += best_tri_weights[0];
        weights[t[1]] += best_tri_weights[1];
        weights[t[2]] += best_tri_weights[2];
        return weights;
    }
    return BarycentricForVertices(vertices_, parameter);
}

bool ParameterSupport::Contains(
    const ParameterVector& parameter,
    float tolerance) const {
    if (tolerance < 0.0f) {
        throw std::runtime_error("ParameterSupport::Contains: tolerance must be nonnegative");
    }
    if (type_ == Type::kTriangulated2D) {
        for (const auto& t : triangles_) {
            std::vector<ParameterVector> tri_vertices = { vertices_[t[0]], vertices_[t[1]], vertices_[t[2]] };
            ParameterSupport tri(tri_vertices);
            if (tri.Contains(parameter, tolerance)) {
                return true;
            }
        }
        return false;
    }
    const std::vector<float> weights = BarycentricWeights(parameter);
    for (const float weight : weights) {
        if (weight < -tolerance || weight > 1.0f + tolerance) {
            return false;
        }
    }
    return true;
}

ParameterVector ParameterSupport::Project(const ParameterVector& parameter) const {
    RequireSameParameterDimension(vertices_.front(), parameter, "ParameterSupport::Project");
    if (!IsFiniteVector(parameter)) {
        throw std::runtime_error("ParameterSupport::Project: parameter must be finite");
    }

    if (type_ == Type::kTriangulated2D) {
        ParameterVector best_point;
        float best_distance = std::numeric_limits<float>::infinity();
        for (const auto& t : triangles_) {
            std::vector<ParameterVector> tri_vertices = { vertices_[t[0]], vertices_[t[1]], vertices_[t[2]] };
            ParameterSupport tri(tri_vertices);
            ParameterVector candidate = tri.Project(parameter);
            float dist = SquaredDistance(parameter, candidate);
            if (dist < best_distance) {
                best_distance = dist;
                best_point = std::move(candidate);
            }
        }
        return best_point;
    }

    if (vertices_.size() > kMaxEnumeratedSimplexVertices) {
        throw std::runtime_error("ParameterSupport::Project: simplex dimension too large");
    }

    ParameterVector best_point;
    float best_distance = std::numeric_limits<float>::infinity();

    // TODO: Enumerate simplex faces for now; replace with an active-set solver if
    // authored parameter spaces ever exceed small PMG dimensions.
    const std::size_t face_count = std::size_t{1} << vertices_.size();
    for (std::size_t mask = 1; mask < face_count; ++mask) {
        std::vector<ParameterVector> face_vertices;
        for (std::size_t vertex = 0; vertex < vertices_.size(); ++vertex) {
            if ((mask & (std::size_t{1} << vertex)) != 0) {
                face_vertices.push_back(vertices_[vertex]);
            }
        }

        std::vector<float> weights;
        try {
            weights = BarycentricForVertices(face_vertices, parameter);
        } catch (const std::runtime_error&) {
            continue;
        }
        if (std::any_of(weights.begin(), weights.end(), [](float weight) {
                return weight < -kWeightTolerance;
            })) {
            continue;
        }

        ParameterVector candidate = Combine(face_vertices, weights);
        const float distance = SquaredDistance(parameter, candidate);
        if (distance < best_distance) {
            best_distance = distance;
            best_point = std::move(candidate);
        }
    }

    if (best_point.empty()) {
        throw std::runtime_error("ParameterSupport::Project: no valid simplex face");
    }
    return best_point;
}

ParameterVector ParameterSupport::SampleUniform(std::mt19937& rng) const {
    if (type_ == Type::kTriangulated2D) {
        std::vector<float> areas;
        areas.reserve(triangles_.size());
        for (const auto& t : triangles_) {
            const ParameterVector& p0 = vertices_[t[0]];
            const ParameterVector& p1 = vertices_[t[1]];
            const ParameterVector& p2 = vertices_[t[2]];
            float cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
            areas.push_back(std::abs(cross) * 0.5f);
        }
        std::discrete_distribution<std::size_t> dist(areas.begin(), areas.end());
        std::size_t tri_idx = dist(rng);
        const auto& t = triangles_[tri_idx];
        std::vector<ParameterVector> tri_vertices = { vertices_[t[0]], vertices_[t[1]], vertices_[t[2]] };
        ParameterSupport tri(tri_vertices);
        return tri.SampleUniform(rng);
    }

    std::exponential_distribution<float> exponential(1.0f);
    std::vector<float> weights(vertices_.size(), 0.0f);
    float sum = 0.0f;
    for (float& weight : weights) {
        weight = exponential(rng);
        sum += weight;
    }
    if (!std::isfinite(sum) || sum <= 0.0f) {
        throw std::runtime_error("ParameterSupport::SampleUniform: invalid random weights");
    }
    for (float& weight : weights) {
        weight /= sum;
    }
    return Combine(vertices_, weights);
}

}  // namespace pmg
