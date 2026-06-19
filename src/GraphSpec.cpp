#include "pmg/GraphSpec.h"

#include "pmg/PmgOfflinePipeline.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

std::string StripComment(const std::string& line) {
    const std::size_t hash = line.find('#');
    if (hash == std::string::npos) {
        return line;
    }
    return line.substr(0, hash);
}

const GraphSpecNode& FindSpecNode(const GraphSpec& spec, const std::string& name) {
    for (const GraphSpecNode& node : spec.nodes) {
        if (node.name == name) {
            return node;
        }
    }
    throw std::runtime_error("GraphSpec: unknown node '" + name + "'");
}

std::string ResolveRelativePath(const std::filesystem::path& base_directory,
                                const std::string& path_text) {
    const std::filesystem::path path(path_text);
    if (path.is_absolute()) {
        return path.string();
    }
    return (base_directory / path).lexically_normal().string();
}

std::vector<std::string> SplitCommaList(const std::string& text) {
    std::vector<std::string> values;
    std::istringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ',')) {
        if (!value.empty()) {
            values.push_back(value);
        }
    }
    return values;
}

ParameterMetric ParseParameterMetric(
    const std::string& metric_name,
    int line_number) {
    if (metric_name == "turn_rate") {
        return ParameterMetric::kTurnRate;
    }
    if (metric_name == "travel_speed") {
        return ParameterMetric::kTravelSpeed;
    }
    if (metric_name == "none") {
        return ParameterMetric::kNone;
    }
    throw std::runtime_error(
        "LoadGraphSpec line " + std::to_string(line_number) +
        ": unknown parameter metric '" + metric_name + "'");
}

bool HasFullSimplexRank(const std::vector<const ParameterVector*>& params,
                        int dimension) {
    constexpr float kRankEpsilon = 1.0e-6f;
    std::vector<std::vector<float>> matrix(
        static_cast<std::size_t>(dimension),
        std::vector<float>(static_cast<std::size_t>(dimension), 0.0f));
    for (int column = 0; column < dimension; ++column) {
        for (int row = 0; row < dimension; ++row) {
            matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)] =
                (*params[static_cast<std::size_t>(column + 1)])[row] -
                (*params.front())[row];
        }
    }

    int rank = 0;
    for (int column = 0; column < dimension; ++column) {
        int pivot = rank;
        for (int row = rank + 1; row < dimension; ++row) {
            if (std::fabs(matrix[static_cast<std::size_t>(row)]
                                [static_cast<std::size_t>(column)]) >
                std::fabs(matrix[static_cast<std::size_t>(pivot)]
                                [static_cast<std::size_t>(column)])) {
                pivot = row;
            }
        }
        if (std::fabs(matrix[static_cast<std::size_t>(pivot)]
                            [static_cast<std::size_t>(column)]) <= kRankEpsilon) {
            continue;
        }
        std::swap(matrix[static_cast<std::size_t>(rank)],
                  matrix[static_cast<std::size_t>(pivot)]);
        const float pivot_value =
            matrix[static_cast<std::size_t>(rank)]
                  [static_cast<std::size_t>(column)];
        for (int row = rank + 1; row < dimension; ++row) {
            const float scale =
                matrix[static_cast<std::size_t>(row)]
                      [static_cast<std::size_t>(column)] /
                pivot_value;
            for (int col = column; col < dimension; ++col) {
                matrix[static_cast<std::size_t>(row)]
                      [static_cast<std::size_t>(col)] -=
                    scale *
                    matrix[static_cast<std::size_t>(rank)]
                          [static_cast<std::size_t>(col)];
            }
        }
        ++rank;
    }
    return rank == dimension;
}

// Checks each node's declared `expect` clauses against its parsed examples so a
// degenerate space (e.g. a "2-D" node that only samples a triangle of corners)
// fails at load instead of building a misleading artifact. Only nodes that
// declare an expectation are inspected.
void ValidateNodeExpectations(const GraphSpec& spec) {
    constexpr int kMaxCornerDimension = 8;
    for (const GraphSpecNode& node : spec.nodes) {
        if (!node.expect_corner_coverage_full && !node.has_expect_spanned_axes) {
            continue;
        }
        std::vector<const ParameterVector*> params;
        for (const GraphSpecExample& example : spec.examples) {
            if (example.node_name == node.name) {
                params.push_back(&example.parameter);
            }
        }
        if (params.empty()) {
            throw std::runtime_error(
                "LoadGraphSpec: node '" + node.name +
                "' declares an expect clause but has no examples");
        }
        const int dimension = node.parameter_dimension;

        if (node.has_expect_spanned_axes) {
            int spanned = 0;
            for (int axis = 0; axis < dimension; ++axis) {
                const float first = (*params.front())[axis];
                for (const ParameterVector* p : params) {
                    if ((*p)[axis] != first) {
                        ++spanned;
                        break;
                    }
                }
            }
            if (spanned != node.expect_spanned_axes) {
                throw std::runtime_error(
                    "LoadGraphSpec: node '" + node.name + "' expects spanned_axes " +
                    std::to_string(node.expect_spanned_axes) +
                    " but its examples span " + std::to_string(spanned));
            }
        }

        if (node.expect_corner_coverage_full) {
            if (dimension > kMaxCornerDimension) {
                throw std::runtime_error(
                    "LoadGraphSpec: node '" + node.name +
                    "' corner_coverage check is unsupported for "
                    "parameter_dimension > 8");
            }
            std::vector<float> low(static_cast<std::size_t>(dimension));
            std::vector<float> high(static_cast<std::size_t>(dimension));
            for (int axis = 0; axis < dimension; ++axis) {
                low[axis] = high[axis] = (*params.front())[axis];
                for (const ParameterVector* p : params) {
                    low[axis] = std::min(low[axis], (*p)[axis]);
                    high[axis] = std::max(high[axis], (*p)[axis]);
                }
            }
            const int corner_count = 1 << dimension;
            for (int mask = 0; mask < corner_count; ++mask) {
                bool corner_sampled = false;
                for (const ParameterVector* p : params) {
                    bool matches = true;
                    for (int axis = 0; axis < dimension; ++axis) {
                        const float want =
                            (mask & (1 << axis)) ? high[axis] : low[axis];
                        if ((*p)[axis] != want) {
                            matches = false;
                            break;
                        }
                    }
                    if (matches) {
                        corner_sampled = true;
                        break;
                    }
                }
                if (!corner_sampled) {
                    throw std::runtime_error(
                        "LoadGraphSpec: node '" + node.name +
                        "' expects corner_coverage full but a corner of its "
                        "parameter box has no example");
                }
            }
        }
    }
}

void ValidateParameterSupport(const GraphSpec& spec) {
    for (const GraphSpecNode& node : spec.nodes) {
        if (!node.parameter_support_simplex) {
            continue;
        }
        std::vector<const ParameterVector*> params;
        for (const GraphSpecExample& example : spec.examples) {
            if (example.node_name == node.name) {
                params.push_back(&example.parameter);
            }
        }
        const int expected_count = node.parameter_dimension + 1;
        if (static_cast<int>(params.size()) != expected_count) {
            throw std::runtime_error(
                "LoadGraphSpec: node '" + node.name +
                "' declares simplex parameter_support but has " +
                std::to_string(params.size()) + " examples; expected " +
                std::to_string(expected_count));
        }
        if (!HasFullSimplexRank(params, node.parameter_dimension)) {
            throw std::runtime_error(
                "LoadGraphSpec: node '" + node.name +
                "' declares simplex parameter_support but its examples are "
                "not affinely independent");
        }
    }
}

void ValidateTriangulatedSupport(const GraphSpec& spec) {
    for (const GraphSpecNode& node : spec.nodes) {
        if (!node.parameter_support_triangulated_2d) {
            continue;
        }
        if (node.parameter_dimension != 2) {
            throw std::runtime_error(
                "LoadGraphSpec: node '" + node.name +
                "' triangulated_2d requires parameter dimension == 2");
        }
        if (node.parameter_triangles.empty()) {
            throw std::runtime_error(
                "LoadGraphSpec: node '" + node.name +
                "' triangulated_2d requires at least one triangle");
        }

        std::vector<const ParameterVector*> params;
        for (const GraphSpecExample& example : spec.examples) {
            if (example.node_name == node.name) {
                params.push_back(&example.parameter);
            }
        }

        std::vector<bool> covered(params.size(), false);
        for (const std::array<int, 3>& t : node.parameter_triangles) {
            for (int idx : t) {
                if (idx < 0 || idx >= static_cast<int>(params.size())) {
                    throw std::runtime_error(
                        "LoadGraphSpec: node '" + node.name +
                        "' triangle index out of bounds");
                }
            }
            if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) {
                throw std::runtime_error(
                    "LoadGraphSpec: node '" + node.name +
                    "' duplicate vertex index inside a triangle");
            }

            const ParameterVector& p0 = *params[static_cast<std::size_t>(t[0])];
            const ParameterVector& p1 = *params[static_cast<std::size_t>(t[1])];
            const ParameterVector& p2 = *params[static_cast<std::size_t>(t[2])];
            float cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
            if (std::abs(cross) <= 1.0e-5f) {
                throw std::runtime_error(
                    "LoadGraphSpec: node '" + node.name +
                    "' triangle area must be > epsilon");
            }

            covered[static_cast<std::size_t>(t[0])] = true;
            covered[static_cast<std::size_t>(t[1])] = true;
            covered[static_cast<std::size_t>(t[2])] = true;
        }

        for (std::size_t i = 0; i < covered.size(); ++i) {
            if (!covered[i]) {
                throw std::runtime_error(
                    "LoadGraphSpec: node '" + node.name +
                    "' example is not covered by any triangle");
            }
        }
    }
}

}  // namespace

GraphSpec LoadGraphSpec(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("LoadGraphSpec: failed to open '" + path + "'");
    }

    const std::filesystem::path base_directory =
        std::filesystem::absolute(std::filesystem::path(path)).parent_path();

    GraphSpec spec;
    std::map<std::string, int> node_dimensions;
    std::string raw_line;
    int line_number = 0;
    while (std::getline(input, raw_line)) {
        ++line_number;
        std::istringstream line(StripComment(raw_line));
        std::string keyword;
        if (!(line >> keyword)) {
            continue;
        }

        if (keyword == "node") {
            GraphSpecNode node;
            if (!(line >> node.name >> node.parameter_dimension)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": expected node <name> <parameter_dimension>");
            }
            if (node.parameter_dimension <= 0) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": parameter_dimension must be positive");
            }
            if (node_dimensions.count(node.name) != 0) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": duplicate node '" + node.name + "'");
            }
            node_dimensions[node.name] = node.parameter_dimension;
            spec.nodes.push_back(node);
            continue;
        }

        if (keyword == "example") {
            GraphSpecExample example;
            if (!(line >> example.node_name)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": expected example <node_name> ...");
            }
            const auto dimension_it = node_dimensions.find(example.node_name);
            if (dimension_it == node_dimensions.end()) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": example references unknown node '" +
                                         example.node_name + "'");
            }
            example.parameter.resize(static_cast<std::size_t>(dimension_it->second));
            for (float& value : example.parameter) {
                if (!(line >> value)) {
                    throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                             ": missing parameter value");
                }
            }
            std::string bvh_path;
            if (!(line >> bvh_path)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": missing BVH path");
            }
            example.bvh_path = ResolveRelativePath(base_directory, bvh_path);
            spec.examples.push_back(example);
            continue;
        }

        if (keyword == "registration") {
            std::string node_name;
            std::string cycle_joint;
            std::string contact_joint_csv;
            int min_contact_frames = 0;
            int dtw_refine = 0;
            if (!(line >> node_name >> cycle_joint >> contact_joint_csv >>
                  min_contact_frames >> dtw_refine)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected registration <node> <cycle_joint|-> "
                    "<contact_joint_csv|-> <min_frames> <dtw:0|1>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) { return node.name == node_name; });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": registration references unknown node '" +
                                         node_name + "'");
            }
            if (node_it->has_registration_config) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate registration for node '" +
                                         node_name + "'");
            }
            if (min_contact_frames <= 0 || (dtw_refine != 0 && dtw_refine != 1)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": registration requires positive min_frames and dtw 0 or 1");
            }
            node_it->has_registration_config = true;
            node_it->cycle_joint = cycle_joint == "-" ? "" : cycle_joint;
            node_it->contact_joints =
                contact_joint_csv == "-" ? std::vector<std::string>{}
                                          : SplitCommaList(contact_joint_csv);
            node_it->min_contact_frames = min_contact_frames;
            node_it->dtw_refine = dtw_refine != 0;
            continue;
        }

        if (keyword == "parameter_metric" || keyword == "parameter_metrics") {
            std::string node_name;
            if (!(line >> node_name)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected parameter metric node name");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) { return node.name == node_name; });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": parameter metric references unknown node '" +
                                         node_name + "'");
            }
            if (node_it->has_parameter_metrics_config) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": duplicate parameter metric declaration for node '" +
                    node_name + "'");
            }

            std::vector<ParameterMetric> metrics;
            std::string metric_name;
            while (line >> metric_name) {
                metrics.push_back(
                    ParseParameterMetric(metric_name, line_number));
            }
            if (metrics.empty()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter metric declaration requires at least one metric");
            }
            if (keyword == "parameter_metric" &&
                node_it->parameter_dimension != 1) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter_metric is the legacy one-dimensional form; "
                    "use parameter_metrics");
            }
            const bool disables_calibration =
                metrics.size() == 1 &&
                metrics.front() == ParameterMetric::kNone;
            if (!disables_calibration &&
                static_cast<int>(metrics.size()) !=
                    node_it->parameter_dimension) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": metric count must match node parameter dimension");
            }
            node_it->has_parameter_metrics_config = true;
            if (!disables_calibration) {
                for (std::size_t metric_index = 0;
                     metric_index < metrics.size(); ++metric_index) {
                    const ParameterMetric metric = metrics[metric_index];
                    if (metric == ParameterMetric::kNone) {
                        throw std::runtime_error(
                            "LoadGraphSpec line " +
                            std::to_string(line_number) +
                            ": none cannot be mixed with active metrics");
                    }
                    for (std::size_t previous_index = 0;
                         previous_index < metric_index; ++previous_index) {
                        if (metrics[previous_index] == metric) {
                            throw std::runtime_error(
                                "LoadGraphSpec line " +
                                std::to_string(line_number) +
                                ": parameter metrics must be unique");
                        }
                    }
                }
                node_it->parameter_metrics = std::move(metrics);
            }
            continue;
        }

        if (keyword == "parameter_calibration") {
            std::string node_name;
            int samples_per_axis = 0;
            if (!(line >> node_name >> samples_per_axis)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected parameter_calibration <node> "
                    "<samples_per_axis>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) {
                    return node.name == node_name;
                });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter_calibration references unknown node '" +
                    node_name + "'");
            }
            if (samples_per_axis < 2) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter_calibration samples_per_axis must be at "
                    "least 2");
            }
            if (node_it->has_calibration_sampling_config) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": duplicate parameter_calibration for node '" +
                    node_name + "'");
            }
            node_it->has_calibration_sampling_config = true;
            node_it->calibration_samples_per_axis = samples_per_axis;
            continue;
        }

        if (keyword == "parameter_support") {
            std::string node_name;
            std::string support_shape;
            if (!(line >> node_name >> support_shape)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected parameter_support <node> <simplex|triangulated_2d>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) {
                    return node.name == node_name;
                });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter_support references unknown node '" +
                    node_name + "'");
            }
            if (support_shape == "simplex") {
                if (node_it->parameter_support_simplex || node_it->parameter_support_triangulated_2d) {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": duplicate parameter_support for node '" + node_name + "'");
                }
                node_it->parameter_support_simplex = true;
            } else if (support_shape == "triangulated_2d") {
                if (node_it->parameter_support_simplex || node_it->parameter_support_triangulated_2d) {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": duplicate parameter_support for node '" + node_name + "'");
                }
                node_it->parameter_support_triangulated_2d = true;
            } else {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter_support only supports 'simplex' and 'triangulated_2d'");
            }
            continue;
        }

        if (keyword == "triangle") {
            std::string node_name;
            int i0 = 0;
            int i1 = 0;
            int i2 = 0;
            if (!(line >> node_name >> i0 >> i1 >> i2)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected triangle <node> <i0> <i1> <i2>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) {
                    return node.name == node_name;
                });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": triangle references unknown node '" + node_name + "'");
            }
            node_it->parameter_triangles.push_back({i0, i1, i2});
            continue;
        }

        if (keyword == "edge") {
            GraphSpecEdge edge;
            if (!(line >> edge.source_node >> edge.target_node)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": expected edge <source_node> <target_node>");
            }
            (void)FindSpecNode(spec, edge.source_node);
            (void)FindSpecNode(spec, edge.target_node);
            spec.edges.push_back(edge);
            continue;
        }

        if (keyword == "edge_config") {
            std::string source_node;
            std::string target_node;
            float good_threshold = 0.0f;
            float bad_threshold = 0.0f;
            int source_samples = 0;
            int target_samples = 0;
            unsigned int seed = 0u;
            if (!(line >> source_node >> target_node >> good_threshold >>
                  bad_threshold >> source_samples >> target_samples >> seed)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected edge_config <source> <target> <tgood> <tbad> "
                    "<source_samples> <target_samples> <seed>");
            }
            (void)FindSpecNode(spec, source_node);
            (void)FindSpecNode(spec, target_node);
            auto edge_it = std::find_if(
                spec.edges.begin(), spec.edges.end(),
                [&](const GraphSpecEdge& edge) {
                    return edge.source_node == source_node &&
                           edge.target_node == target_node;
                });
            if (edge_it == spec.edges.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": edge_config requires a preceding matching edge");
            }
            if (edge_it->has_threshold_config) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate edge_config");
            }
            // Merge only the fields this line sets, preserving any distance-grid
            // phase range already installed by an edge_phase_range line.
            edge_it->has_build_config = true;
            edge_it->has_threshold_config = true;
            edge_it->build_config.good_transition_threshold = good_threshold;
            edge_it->build_config.bad_transition_threshold = bad_threshold;
            edge_it->build_config.source_sample_count = source_samples;
            edge_it->build_config.target_sample_count = target_samples;
            edge_it->build_config.seed = seed;
            continue;
        }

        if (keyword == "edge_phase_range") {
            std::string source_node;
            std::string target_node;
            float source_start = 0.0f;
            float source_end = 0.0f;
            float target_start = 0.0f;
            float target_end = 0.0f;
            if (!(line >> source_node >> target_node >> source_start >>
                  source_end >> target_start >> target_end)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected edge_phase_range <source> <target> <src_start> "
                    "<src_end> <tgt_start> <tgt_end>");
            }
            const auto valid_range = [](float start, float end) {
                return start >= 0.0f && end <= 1.0f && start < end;
            };
            if (!valid_range(source_start, source_end) ||
                !valid_range(target_start, target_end)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": edge_phase_range values must satisfy 0 <= start < end <= 1");
            }
            (void)FindSpecNode(spec, source_node);
            (void)FindSpecNode(spec, target_node);
            auto edge_it = std::find_if(
                spec.edges.begin(), spec.edges.end(),
                [&](const GraphSpecEdge& edge) {
                    return edge.source_node == source_node &&
                           edge.target_node == target_node;
                });
            if (edge_it == spec.edges.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": edge_phase_range requires a preceding matching edge");
            }
            if (edge_it->has_phase_range) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate edge_phase_range");
            }
            edge_it->has_build_config = true;
            edge_it->has_phase_range = true;
            edge_it->build_config.distance_grid.source_phase_start = source_start;
            edge_it->build_config.distance_grid.source_phase_end = source_end;
            edge_it->build_config.distance_grid.target_phase_start = target_start;
            edge_it->build_config.distance_grid.target_phase_end = target_end;
            continue;
        }

        if (keyword == "edge_metric") {
            std::string source_node;
            std::string target_node;
            std::string metric_name;
            if (!(line >> source_node >> target_node >> metric_name)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected edge_metric <source> <target> <metric>");
            }
            (void)FindSpecNode(spec, source_node);
            (void)FindSpecNode(spec, target_node);
            auto edge_it = std::find_if(
                spec.edges.begin(), spec.edges.end(),
                [&](const GraphSpecEdge& edge) {
                    return edge.source_node == source_node &&
                           edge.target_node == target_node;
                });
            if (edge_it == spec.edges.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": edge_metric requires a preceding matching edge");
            }
            if (edge_it->has_metric_type) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate edge_metric");
            }
            edge_it->has_build_config = true;
            edge_it->has_metric_type = true;
            edge_it->build_config.transition_metric_type =
                ParseTransitionMetricType(metric_name);
            continue;
        }

        if (keyword == "edge_metric_config") {
            std::string source_node;
            std::string target_node;
            TransitionMetricConfig metric_config;
            if (!(line >> source_node >> target_node
                  >> metric_config.position_weight
                  >> metric_config.velocity_weight
                  >> metric_config.acceleration_weight
                  >> metric_config.root_motion_weight
                  >> metric_config.foot_contact_weight
                  >> metric_config.position_scale
                  >> metric_config.velocity_scale
                  >> metric_config.acceleration_scale
                  >> metric_config.root_speed_scale
                  >> metric_config.yaw_rate_scale
                  >> metric_config.root_displacement_weight
                  >> metric_config.root_speed_weight
                  >> metric_config.root_yaw_rate_weight
                  >> metric_config.foot_mismatch_penalty)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected edge_metric_config <source> <target> "
                    "<position_weight> <velocity_weight> "
                    "<acceleration_weight> <root_motion_weight> "
                    "<foot_contact_weight> <position_scale> <velocity_scale> "
                    "<acceleration_scale> <root_speed_scale> "
                    "<yaw_rate_scale> <root_displacement_weight> "
                    "<root_speed_weight> <root_yaw_rate_weight> "
                    "<foot_mismatch_penalty>");
            }
            (void)FindSpecNode(spec, source_node);
            (void)FindSpecNode(spec, target_node);
            auto edge_it = std::find_if(
                spec.edges.begin(), spec.edges.end(),
                [&](const GraphSpecEdge& edge) {
                    return edge.source_node == source_node &&
                           edge.target_node == target_node;
                });
            if (edge_it == spec.edges.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": edge_metric_config requires a preceding matching edge");
            }
            if (edge_it->has_metric_config) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate edge_metric_config");
            }
            edge_it->has_build_config = true;
            edge_it->has_metric_config = true;
            edge_it->build_config.transition_metric = metric_config;
            continue;
        }

        if (keyword == "expect") {
            std::string node_name;
            std::string property;
            if (!(line >> node_name >> property)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected expect <node> <property> <value>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) { return node.name == node_name; });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": expect references unknown node '" +
                                         node_name + "'");
            }
            if (property == "corner_coverage") {
                std::string value;
                if (!(line >> value) || value != "full") {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": expect corner_coverage requires the value 'full'");
                }
                if (node_it->expect_corner_coverage_full) {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": duplicate expect corner_coverage for node '" +
                        node_name + "'");
                }
                node_it->expect_corner_coverage_full = true;
            } else if (property == "spanned_axes") {
                int count = -1;
                if (!(line >> count)) {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": expect spanned_axes requires an integer count");
                }
                if (count < 0 || count > node_it->parameter_dimension) {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": expect spanned_axes count must be in "
                        "[0, parameter_dimension]");
                }
                if (node_it->has_expect_spanned_axes) {
                    throw std::runtime_error(
                        "LoadGraphSpec line " + std::to_string(line_number) +
                        ": duplicate expect spanned_axes for node '" +
                        node_name + "'");
                }
                node_it->has_expect_spanned_axes = true;
                node_it->expect_spanned_axes = count;
            } else {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": unknown expect property '" + property + "'");
            }
            continue;
        }

        throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                 ": unknown keyword '" + keyword + "'");
    }

    if (spec.nodes.empty()) {
        throw std::runtime_error("LoadGraphSpec: spec contains no nodes");
    }
    for (const GraphSpecNode& node : spec.nodes) {
        if (node.has_calibration_sampling_config &&
            node.parameter_metrics.empty()) {
            throw std::runtime_error(
                "LoadGraphSpec: node '" + node.name +
                "' configures parameter_calibration without active metrics");
        }
    }
    ValidateNodeExpectations(spec);
    ValidateParameterSupport(spec);
    ValidateTriangulatedSupport(spec);
    return spec;
}

BuiltPmgArtifact BuildPmgArtifactFromSpec(
    const GraphSpec& spec,
    const ArtifactBuildConfig& config) {
    return BuildPmgOfflinePipeline(spec, config);
}

}  // namespace pmg
