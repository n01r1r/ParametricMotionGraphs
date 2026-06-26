#include "PmgCommandModules.h"

#include "pmg/Diagnostics.h"
#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/ContactDetection.h"
#include "pmg/CyclicContinuity.h"
#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/MotionSpacePreparation.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/PoseBlend.h"
#include "pmg/RuntimeController.h"
#include "pmg/RootCanonicalization.h"
#include "pmg/SkeletonCompatibility.h"
#include "pmg/TransitionQuality.h"
#include "pmg/TransitionDiagnostics.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>


namespace {


float HorizontalLength(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

std::vector<std::string> SplitCommaList(const std::string& text) {
    std::vector<std::string> items;
    std::istringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (!item.empty()) {
            items.push_back(item);
        }
    }
    return items;
}

void RequireMatchingProductionRegistration(
    const pmg::NodeRegistrationMetadata& registration,
    const std::string& node_name,
    const std::string& cycle_joint,
    bool cycle_joint_set,
    const std::vector<std::string>& contact_joints,
    bool contact_joints_set,
    int min_contact_frames,
    bool min_contact_frames_set,
    const char* command) {
    if (cycle_joint_set && registration.cycle_joint != cycle_joint) {
        throw std::runtime_error(
            std::string(command) + ": --cycle-joint does not match node '" +
            node_name + "' production registration");
    }
    if (contact_joints_set &&
        registration.contact_joints != contact_joints) {
        throw std::runtime_error(
            std::string(command) +
            ": --contact-joints does not match node '" + node_name +
            "' production registration");
    }
    if (min_contact_frames_set &&
        registration.min_contact_frames != min_contact_frames) {
        throw std::runtime_error(
            std::string(command) +
            ": --min-contact-frames does not match node '" + node_name +
            "' production registration");
    }
}

struct ValidateGraphOptions {
    std::string spec_path;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    bool contact_joints_set = false;
    bool cycle_joint_set = false;
    int min_contact_frames = 3;
    bool min_contact_frames_set = false;
    pmg::PmgBuilderConfig builder;
    bool tgood_set = false;
    bool tbad_set = false;
    bool source_samples_set = false;
    bool target_samples_set = false;
    bool seed_set = false;
    // Assert thresholds on the registered build; negative = report only.
    int min_edge_samples = -1;
    float min_good_fraction = -1.0f;
    bool assert_no_regression = false;
    // Negative disables the comparison. A value of 1.0 requires no increase.
    float max_preparation_min_distance_ratio = -1.0f;
};

struct CyclicAuditOptions {
    std::string spec_path;
    std::string output_csv;
    std::string output_md;
    float frames_per_second = 30.0f;
};

struct CyclicAuditRow {
    std::string node;
    std::string sample_type;
    pmg::ParameterVector parameter;
    std::string source_clip;
    int num_frames = 0;
    pmg::CyclicContinuityRecord record;
};

std::string JoinParameter(const pmg::ParameterVector& parameter) {
    std::ostringstream cell;
    for (std::size_t axis = 0; axis < parameter.size(); ++axis) {
        if (axis != 0) {
            cell << '|';
        }
        cell << parameter[axis];
    }
    return cell.str();
}

std::string EscapeCsvCell(const std::string& cell) {
    if (cell.find_first_of(",\"\n") == std::string::npos) {
        return cell;
    }
    std::string escaped = "\"";
    for (const char ch : cell) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped += ch;
        }
    }
    escaped += '"';
    return escaped;
}

std::string CurrentGitCommit() {
    std::array<char, 128> buffer{};
    std::string output;
#ifdef _WIN32
    FILE* pipe = _popen("git rev-parse --short HEAD", "r");
#else
    FILE* pipe = popen("git rev-parse --short HEAD", "r");
#endif
    if (pipe == nullptr) {
        return "unknown";
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) !=
           nullptr) {
        output += buffer.data();
    }
#ifdef _WIN32
    _pclose(pipe);
#else
    pclose(pipe);
#endif
    while (!output.empty() &&
           (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }
    return output.empty() ? "unknown" : output;
}

std::string CommandLineString(int argc, char** argv) {
    std::ostringstream command;
    for (int index = 0; index < argc; ++index) {
        if (index != 0) {
            command << ' ';
        }
        command << argv[index];
    }
    return command.str();
}

int ResolveJointIndex(
    const pmg::Skeleton& skeleton,
    const std::string& joint_name) {
    for (int joint_index = 0; joint_index < skeleton.NumJoints();
         ++joint_index) {
        if (skeleton.joints[static_cast<std::size_t>(joint_index)].name ==
            joint_name) {
            return joint_index;
        }
    }
    return -1;
}

pmg::CyclicContinuityContext CyclicContextForNode(
    const pmg::Skeleton& skeleton,
    const pmg::PreparedMotionSpace& prepared_node) {
    pmg::CyclicContinuityContext context;
    for (const std::string& joint_name :
         prepared_node.registration.contact_joints) {
        std::string lower_name = joint_name;
        std::transform(
            lower_name.begin(), lower_name.end(), lower_name.begin(),
            [](unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        if (lower_name.find("left") != std::string::npos) {
            context.left_foot_joint = ResolveJointIndex(skeleton, joint_name);
        } else if (lower_name.find("right") != std::string::npos) {
            context.right_foot_joint = ResolveJointIndex(skeleton, joint_name);
        }
    }
    context.contact_settings = prepared_node.contact_settings;
    return context;
}

std::vector<int> ResolveContactJointIndices(
    const pmg::Skeleton& skeleton,
    const std::vector<std::string>& joint_names,
    const char* context) {
    std::vector<int> joint_indices;
    joint_indices.reserve(joint_names.size());
    for (const std::string& joint_name : joint_names) {
        const int joint_index = ResolveJointIndex(skeleton, joint_name);
        if (joint_index < 0) {
            throw std::runtime_error(
                std::string(context) + ": unknown joint '" + joint_name + "'");
        }
        joint_indices.push_back(joint_index);
    }
    return joint_indices;
}

std::vector<pmg::ParameterVector> GeneratedAuditParameters(
    const pmg::ParametricMotionSpace& space) {
    const int dimension = space.ParameterDimension();
    const std::vector<pmg::ParameterVector> examples =
        space.ExampleParameters();
    if (examples.empty()) {
        throw std::runtime_error(
            "GeneratedAuditParameters: space has no examples");
    }
    pmg::ParameterVector min_corner(static_cast<std::size_t>(dimension),
                                    std::numeric_limits<float>::infinity());
    pmg::ParameterVector max_corner(static_cast<std::size_t>(dimension),
                                    -std::numeric_limits<float>::infinity());
    for (const pmg::ParameterVector& parameter : examples) {
        pmg::RequireSameParameterDimension(
            min_corner, parameter, "GeneratedAuditParameters");
        for (int axis = 0; axis < dimension; ++axis) {
            min_corner[static_cast<std::size_t>(axis)] = std::min(
                min_corner[static_cast<std::size_t>(axis)],
                parameter[static_cast<std::size_t>(axis)]);
            max_corner[static_cast<std::size_t>(axis)] = std::max(
                max_corner[static_cast<std::size_t>(axis)],
                parameter[static_cast<std::size_t>(axis)]);
        }
    }

    std::vector<pmg::ParameterVector> samples;

    if (dimension == 1) {
        const float min_value = min_corner[0];
        const float max_value = max_corner[0];
        samples.push_back({min_value});
        if (std::abs(max_value - min_value) > 1.0e-6f) {
            samples.push_back({0.5f * (min_value + max_value)});
            samples.push_back({max_value});
        }
        return samples;
    }

    if (dimension == 2) {
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                samples.push_back({
                    x == 0 ? min_corner[0] : max_corner[0],
                    y == 0 ? min_corner[1] : max_corner[1],
                });
            }
        }
        samples.push_back({
            0.5f * (min_corner[0] + max_corner[0]),
            0.5f * (min_corner[1] + max_corner[1]),
        });
        return samples;
    }

    pmg::ParameterVector center(static_cast<std::size_t>(dimension), 0.0f);
    for (int axis = 0; axis < dimension; ++axis) {
        center[static_cast<std::size_t>(axis)] =
            0.5f * (min_corner[static_cast<std::size_t>(axis)] +
                    max_corner[static_cast<std::size_t>(axis)]);
    }
    samples.push_back(center);
    return samples;
}

void WriteCyclicAuditCsv(
    const std::string& path,
    const std::vector<CyclicAuditRow>& rows) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error("--audit-cyclic-continuity: cannot write CSV");
    }

    csv << "node,sample_type,parameter,source_clip,num_frames,"
           "cycle_delta_dx,cycle_delta_dz,cycle_delta_yaw,"
           "seam_step,median_step,seam_step_ratio,"
           "pre_root_speed,seam_root_speed,post_root_speed,root_speed_ratio,"
           "pre_yaw_rate,seam_yaw_rate,post_yaw_rate,yaw_rate_ratio,"
           "left_foot_drift,right_foot_drift,max_contact_drift,"
           "has_contact_evidence,contact_state_matches,classification\n";
    for (const CyclicAuditRow& row : rows) {
        const pmg::CyclicContinuityRecord& r = row.record;
        csv << EscapeCsvCell(row.node) << ','
            << EscapeCsvCell(row.sample_type) << ','
            << EscapeCsvCell(JoinParameter(row.parameter)) << ','
            << EscapeCsvCell(row.source_clip) << ',' << row.num_frames << ','
            << r.cycle_delta.dx << ',' << r.cycle_delta.dz << ','
            << r.cycle_delta.yaw << ',' << r.seam_step << ','
            << r.median_step << ',' << r.seam_step_ratio << ','
            << r.pre_root_speed << ',' << r.seam_root_speed << ','
            << r.post_root_speed << ',' << r.root_speed_ratio << ','
            << r.pre_yaw_rate << ',' << r.seam_yaw_rate << ','
            << r.post_yaw_rate << ',' << r.yaw_rate_ratio << ','
            << r.left_foot_drift << ',' << r.right_foot_drift << ','
            << r.max_contact_drift << ','
            << (r.has_contact_evidence ? 1 : 0) << ','
            << (r.contact_state_matches ? 1 : 0) << ','
            << pmg::CyclicContinuityClassificationName(r.classification)
            << '\n';
    }
}

void WriteCyclicAuditMarkdown(
    const std::string& path,
    const CyclicAuditOptions& options,
    const std::string& command_line,
    const std::vector<CyclicAuditRow>& rows) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error("--audit-cyclic-continuity: cannot write markdown");
    }

    md << "# Cyclic Continuity Audit\n\n";
    md << "- Commit: `" << CurrentGitCommit() << "`\n";
    md << "- Command: `" << command_line << "`\n";
    md << "- Spec: `" << options.spec_path << "`\n";
    md << "- Sampling rate: " << options.frames_per_second << " fps\n\n";

    md << "## Summary By Node\n\n";
    md << "| Node | Samples | Strong | Weak pose | Weak root speed | "
          "Weak yaw rate | Weak contact | Insufficient |\n";
    md << "|---|---:|---:|---:|---:|---:|---:|---:|\n";
    std::map<std::string, std::array<int, 6>> counts_by_node;
    for (const CyclicAuditRow& row : rows) {
        auto& counts = counts_by_node[row.node];
        const int index = static_cast<int>(row.record.classification);
        counts[static_cast<std::size_t>(index)] += 1;
    }
    for (const auto& [node, counts] : counts_by_node) {
        int total = 0;
        for (const int count : counts) {
            total += count;
        }
        md << "| " << node << " | " << total << " | "
           << counts[0] << " | " << counts[1] << " | "
           << counts[2] << " | " << counts[3] << " | "
           << counts[4] << " | " << counts[5] << " |\n";
    }

    md << "\n## Per Sample\n\n";
    md << "| Node | Type | Parameter | Clip | Frames | Seam ratio | "
          "Root ratio | Yaw ratio | Contact drift | Classification |\n";
    md << "|---|---|---|---|---:|---:|---:|---:|---:|---|\n";
    for (const CyclicAuditRow& row : rows) {
        const pmg::CyclicContinuityRecord& r = row.record;
        md << "| " << row.node << " | " << row.sample_type << " | `"
           << JoinParameter(row.parameter) << "` | `" << row.source_clip
           << "` | " << row.num_frames << " | " << r.seam_step_ratio
           << " | " << r.root_speed_ratio << " | " << r.yaw_rate_ratio
           << " | " << r.max_contact_drift << " | "
           << pmg::CyclicContinuityClassificationName(r.classification)
           << " |\n";
    }

    std::vector<CyclicAuditRow> ranked = rows;
    std::sort(
        ranked.begin(), ranked.end(),
        [](const CyclicAuditRow& left, const CyclicAuditRow& right) {
            const auto worst = [](const CyclicAuditRow& row) {
                return std::max({
                    row.record.seam_step_ratio,
                    row.record.root_speed_ratio,
                    row.record.yaw_rate_ratio,
                });
            };
            return worst(left) < worst(right);
        });

    md << "\n## Strongest Cyclic Anchors\n\n";
    for (std::size_t index = 0; index < std::min<std::size_t>(3, ranked.size());
         ++index) {
        const CyclicAuditRow& row = ranked[index];
        md << "- `" << row.node << "` " << row.sample_type << " `"
           << JoinParameter(row.parameter) << "`: "
           << pmg::CyclicContinuityClassificationName(
                  row.record.classification)
           << ", seam ratio " << row.record.seam_step_ratio
           << ", root ratio " << row.record.root_speed_ratio
           << ", yaw ratio " << row.record.yaw_rate_ratio << "\n";
    }

    md << "\n## Weakest Cyclic Anchors\n\n";
    for (std::size_t offset = 0;
         offset < std::min<std::size_t>(3, ranked.size()); ++offset) {
        const CyclicAuditRow& row = ranked[ranked.size() - 1 - offset];
        md << "- `" << row.node << "` " << row.sample_type << " `"
           << JoinParameter(row.parameter) << "`: "
           << pmg::CyclicContinuityClassificationName(
                  row.record.classification)
           << ", seam ratio " << row.record.seam_step_ratio
           << ", root ratio " << row.record.root_speed_ratio
           << ", yaw ratio " << row.record.yaw_rate_ratio << "\n";
    }

    const bool all_strong = std::all_of(
        rows.begin(), rows.end(), [](const CyclicAuditRow& row) {
            return row.record.classification ==
                   pmg::CyclicContinuityClassification::kStrong;
        });
    md << "\n## Conclusion\n\n";
    md << (all_strong
               ? "Every audited cyclic sample classified strong under this "
                 "local seam diagnostic."
               : "At least one audited cyclic sample classified weak, so the "
                 "current corpus should not be described as uniformly strong "
                 "for PMG cyclic self-edge streaming.")
       << " This diagnostic measures local seam continuity only; it does not "
          "claim perceptual smoothness.\n";
}

int CyclicAuditCommand(
    const CyclicAuditOptions& options,
    const std::string& command_line) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    pmg::MotionSpacePreparationConfig preparation_config;
    preparation_config.calibration_frames_per_second =
        options.frames_per_second;
    const pmg::PreparedMotionSpaces prepared =
        pmg::PrepareMotionSpaces(spec, preparation_config);

    std::vector<CyclicAuditRow> rows;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        const pmg::PreparedMotionSpace& prepared_node =
            prepared.Node(node.name);
        if (prepared_node.registration.cycle_joint.empty()) {
            continue;
        }

        const pmg::CyclicContinuityContext context =
            CyclicContextForNode(prepared.skeleton, prepared_node);
        for (std::size_t example_index = 0;
             example_index < prepared_node.authored.Examples().size();
             ++example_index) {
            const pmg::ExampleMotion& example =
                prepared_node.authored.Examples()[example_index];
            CyclicAuditRow row;
            row.node = node.name;
            row.sample_type = "authored";
            row.parameter = example.parameter;
            row.source_clip =
                example.clip.name.empty()
                    ? "authored_example_" + std::to_string(example_index)
                    : example.clip.name;
            row.num_frames = example.clip.NumFrames();
            row.record = pmg::MeasureCyclicContinuity(
                prepared.skeleton, example.clip, context);
            rows.push_back(std::move(row));
        }

        for (const pmg::ParameterVector& parameter :
             GeneratedAuditParameters(prepared_node.production)) {
            const pmg::MotionClip clip =
                prepared_node.production.GenerateClip(
                    parameter, options.frames_per_second);
            CyclicAuditRow row;
            row.node = node.name;
            row.sample_type = "generated";
            row.parameter = parameter;
            row.source_clip = "";
            row.num_frames = clip.NumFrames();
            row.record = pmg::MeasureCyclicContinuity(
                prepared.skeleton, clip, context);
            rows.push_back(std::move(row));
        }
    }

    WriteCyclicAuditCsv(options.output_csv, rows);
    WriteCyclicAuditMarkdown(options.output_md, options, command_line, rows);

    std::cout << "cyclic_audit_rows=" << rows.size() << "\n";
    std::cout << "cyclic_audit_csv=" << options.output_csv << "\n";
    std::cout << "cyclic_audit_md=" << options.output_md << "\n";
    return 0;
}

float HorizontalDistance(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

std::string ParameterCsv(const pmg::ParameterVector& parameter) {
    std::ostringstream out;
    out << '"';
    for (std::size_t i = 0; i < parameter.size(); ++i) {
        if (i > 0) {
            out << ' ';
        }
        out << parameter[i];
    }
    out << '"';
    return out.str();
}

std::string ParameterMd(const pmg::ParameterVector& parameter) {
    std::ostringstream out;
    out << '(';
    for (std::size_t i = 0; i < parameter.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << parameter[i];
    }
    out << ')';
    return out.str();
}

std::string Vec3Csv(const pmg::Vec3& v) {
    std::ostringstream out;
    out << '"' << v.x << ' ' << v.y << ' ' << v.z << '"';
    return out.str();
}

std::string PoseSeamClass(
    float distance,
    const pmg::PmgBuilderConfig& config) {
    if (!std::isfinite(distance)) {
        return "EMPTY";
    }
    if (distance <= config.good_transition_threshold) {
        return "GOOD";
    }
    if (distance >= config.bad_transition_threshold) {
        return "BAD";
    }
    return "NEUTRAL";
}

struct ReachableRegionAuditOptions {
    std::string pmg_path;
    std::string output_csv;
    std::string output_md;
    std::string output_dir;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 11;
};

struct TransitionMontageAuditOptions {
    std::string pmg_path;
    std::string output_md;
    std::string output_dir;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 9;
};

struct ContactTransitionAuditOptions {
    std::string pmg_path;
    std::string output_csv;
    std::string output_md;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 9;
};

struct TransitionAcceptanceConsistencyAuditOptions {
    std::string pmg_path;
    std::string output_csv;
    std::string output_md;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 9;
    pmg::TransitionQualityGateConfig quality_gate{
        true, 1.5f, 50.0f, 2.0f, false, 2.0f};
};

struct TransitionProbeOptions {
    std::string pmg_path;
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector target_parameter;
    std::string output_csv;
    std::string output_md;
    std::string source_node;
    std::string target_node;
    int frames_before = 3;
    int frames_after = 3;
    std::string expected_decision;
    pmg::TransitionQualityGateConfig quality_gate{
        true, 1.5f, 50.0f, 2.0f, false, 2.0f};
};

struct SkeletonAuditOptions {
    std::string bvh_path;
    std::filesystem::path out_dir;
};

struct LoopAuditOptions {
    std::string bvh_path;
    std::filesystem::path out_dir;
    int start_frame = -1;
    int end_frame = -1;
};

struct TransitionPopAuditOptions {
    std::string pmg_path;
    std::filesystem::path out_dir;
    int worst_k = 8;
};

struct ArtifactLoopSweepRow {
    std::string node_name;
    std::string sample_type;
    pmg::ParameterVector parameter;
    std::string source_clip;
    int start_frame = 0;
    int end_frame = -1;
    pmg::LoopAuditReport loop;
    std::string verdict;
};

struct MotionSpaceRegistrationAuditOptions {
    std::string pmg_path;
    std::filesystem::path out_dir;
};

struct RegistrationAnchorRow {
    std::string node_name;
    int anchor_id = -1;
    pmg::ParameterVector parameter;
    std::string source_bvh_path;
    int start_frame = 0;
    int end_frame = -1;
    int frame_count = 0;
    float duration_seconds = 0.0f;
    pmg::LoopAuditReport loop;
    std::string start_contact_state;
    std::string end_contact_state;
    pmg::Vec3 root_displacement{};
    float heading_displacement = 0.0f;
    std::string verdict;
};

struct RegistrationPairwiseRow {
    std::string node_name;
    int source_anchor_id = -1;
    int target_anchor_id = -1;
    float parameter_distance = 0.0f;
    float start_pose_distance = 0.0f;
    float end_pose_distance = 0.0f;
    bool contact_phase_match = false;
    float root_delta_difference = 0.0f;
    float heading_delta_difference = 0.0f;
    bool duration_frame_match = false;
};

struct RegistrationGeneratedRow {
    std::string node_name;
    std::string sample_type;
    pmg::ParameterVector parameter;
    pmg::LoopAuditReport loop;
    std::string verdict;
    std::vector<int> contributing_anchor_ids;
    std::vector<float> blend_weights;
    std::vector<std::string> contributing_source_bvh_paths;
    std::vector<std::string> contributing_anchor_verdicts;
    std::vector<float> contributing_anchor_cycle_scores;
    std::vector<int> contributing_anchor_contact_mismatch;
    std::vector<std::string> contributing_anchor_start_contact_state;
    std::vector<std::string> contributing_anchor_end_contact_state;
    std::vector<pmg::Vec3> contributing_anchor_root_deltas;
    std::vector<float> contributing_anchor_heading_deltas;
};

enum class RegistrationRootCause {
    kBadSourceAnchorLoop,
    kAnchorPhaseMismatch,
    kRootTrajectoryBlendFailure,
    kInterpolationWeightFailure,
    kMixedFailure,
    kUnknown,
};

struct MotionSpaceRegistrationAuditData {
    std::vector<RegistrationAnchorRow> anchor_rows;
    std::vector<RegistrationPairwiseRow> pair_rows;
    std::vector<RegistrationGeneratedRow> generated_rows;
    RegistrationRootCause root_cause = RegistrationRootCause::kUnknown;
};

float ParameterDistance(
    const pmg::ParameterVector& left,
    const pmg::ParameterVector& right) {
    pmg::RequireSameParameterDimension(left, right, "ParameterDistance");
    float sum = 0.0f;
    for (std::size_t axis = 0; axis < left.size(); ++axis) {
        const float delta = left[axis] - right[axis];
        sum += delta * delta;
    }
    return std::sqrt(sum);
}

float MeanPoseDistance(
    const pmg::Skeleton& skeleton,
    const pmg::Pose& left,
    const pmg::Pose& right) {
    const std::vector<pmg::Vec3> left_positions =
        pmg::ComputeJointWorldPositions(skeleton, left);
    const std::vector<pmg::Vec3> right_positions =
        pmg::ComputeJointWorldPositions(skeleton, right);
    float distance_sum = 0.0f;
    for (std::size_t joint_index = 0; joint_index < left_positions.size();
         ++joint_index) {
        distance_sum +=
            (right_positions[joint_index] - left_positions[joint_index]).Norm();
    }
    return distance_sum / static_cast<float>(left_positions.size());
}

std::vector<int> ResolveRegistrationAuditFootJoints(
    const pmg::Skeleton& skeleton) {
    const std::array<std::string, 4> preferred_names{
        "LeftAnkle", "RightAnkle", "LeftFoot", "RightFoot"};
    std::vector<int> joints;
    for (const std::string& joint_name : preferred_names) {
        const int joint_index = ResolveJointIndex(skeleton, joint_name);
        if (joint_index >= 0) {
            joints.push_back(joint_index);
        }
    }
    return joints;
}

bool ContactActiveAtFrame(
    const std::vector<pmg::ContactInterval>& contacts,
    int joint_index,
    int frame_index) {
    for (const pmg::ContactInterval& interval : contacts) {
        if (interval.joint_index == joint_index &&
            frame_index >= interval.first_frame &&
            frame_index <= interval.last_frame) {
            return true;
        }
    }
    return false;
}

std::string ClipContactStateSummary(
    const std::vector<pmg::ContactInterval>& contacts,
    const std::vector<int>& contact_joints,
    int frame_index) {
    if (contact_joints.empty()) {
        return "n/a";
    }
    std::ostringstream out;
    for (std::size_t joint = 0; joint < contact_joints.size(); ++joint) {
        if (joint > 0) {
            out << ';';
        }
        out << (joint == 0 ? "L=" : "R=")
            << (ContactActiveAtFrame(contacts, contact_joints[joint], frame_index)
                    ? "contact"
                    : "swing");
    }
    return out.str();
}

std::string JoinIntList(const std::vector<int>& values) {
    std::ostringstream out;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ';';
        }
        out << values[index];
    }
    return out.str();
}

std::string JoinFloatList(const std::vector<float>& values) {
    std::ostringstream out;
    out << std::setprecision(6);
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ';';
        }
        out << values[index];
    }
    return out.str();
}

std::string JoinStringList(const std::vector<std::string>& values) {
    std::ostringstream out;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ';';
        }
        out << values[index];
    }
    return out.str();
}

std::string FormatSegmentLabel(
    const std::string& source_bvh_path,
    int start_frame,
    int end_frame) {
    std::ostringstream out;
    out << source_bvh_path;
    if (end_frame >= 0) {
        out << " [" << start_frame << ", " << end_frame << "]";
    }
    return out.str();
}

std::string JoinVec3List(const std::vector<pmg::Vec3>& values) {
    std::ostringstream out;
    out << std::setprecision(6);
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ';';
        }
        out << '[' << values[index].x << ' ' << values[index].y << ' '
            << values[index].z << ']';
    }
    return out.str();
}

const char* RegistrationRootCauseName(RegistrationRootCause cause) {
    switch (cause) {
        case RegistrationRootCause::kBadSourceAnchorLoop:
            return "bad_source_anchor_loop";
        case RegistrationRootCause::kAnchorPhaseMismatch:
            return "anchor_phase_mismatch";
        case RegistrationRootCause::kRootTrajectoryBlendFailure:
            return "root_trajectory_blend_failure";
        case RegistrationRootCause::kInterpolationWeightFailure:
            return "interpolation_weight_failure";
        case RegistrationRootCause::kMixedFailure:
            return "mixed_failure";
        case RegistrationRootCause::kUnknown:
            return "unknown";
    }
    throw std::runtime_error("RegistrationRootCauseName: unsupported cause");
}

std::string LoopSweepVerdict(const pmg::LoopAuditReport& loop) {
    int violations = 0;
    violations += loop.root_normalized_start_end_distance > 1.0f ? 1 : 0;
    violations += loop.root_velocity_discontinuity > 1.0f ? 1 : 0;
    violations += loop.root_yaw_discontinuity > 0.35f ? 1 : 0;
    violations += loop.foot_contact_mismatch > 0 ? 1 : 0;
    if (violations == 0) {
        return "pass";
    }
    if (violations == 1) {
        return "borderline";
    }
    return "fail";
}

void WriteArtifactLoopSweepCsv(
    const std::filesystem::path& path,
    const std::vector<ArtifactLoopSweepRow>& rows) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "audit-transition-pop: cannot write generated_loop_sweep.csv");
    }
    csv << "node_name,sample_type,parameter,source_bvh_path,start_frame,end_frame,frame_count,"
           "root_normalized_loop_pose_distance,root_velocity_seam,yaw_seam,"
           "contact_mismatch,cycle_score,verdict\n";
    for (const ArtifactLoopSweepRow& row : rows) {
        csv << EscapeCsvCell(row.node_name) << ','
            << EscapeCsvCell(row.sample_type) << ','
            << ParameterCsv(row.parameter) << ','
            << EscapeCsvCell(row.source_clip) << ','
            << row.start_frame << ','
            << row.end_frame << ','
            << row.loop.frame_count << ','
            << row.loop.root_normalized_start_end_distance << ','
            << row.loop.root_velocity_discontinuity << ','
            << row.loop.root_yaw_discontinuity << ','
            << row.loop.foot_contact_mismatch << ','
            << row.loop.cycle_score << ','
            << row.verdict << "\n";
    }
}

void WriteArtifactLoopSweepMarkdown(
    const std::filesystem::path& path,
    const std::vector<ArtifactLoopSweepRow>& rows) {
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "audit-transition-pop: cannot write generated_loop_sweep.md");
    }
    const int pass_count = static_cast<int>(std::count_if(
        rows.begin(), rows.end(), [](const ArtifactLoopSweepRow& row) {
            return row.verdict == "pass";
        }));
    const int borderline_count = static_cast<int>(std::count_if(
        rows.begin(), rows.end(), [](const ArtifactLoopSweepRow& row) {
            return row.verdict == "borderline";
        }));
    const int fail_count = static_cast<int>(std::count_if(
        rows.begin(), rows.end(), [](const ArtifactLoopSweepRow& row) {
            return row.verdict == "fail";
        }));

    md << "# Generated Loop Sweep\n\n";
    md << "- Rows: `" << rows.size() << "`\n";
    md << "- Pass: `" << pass_count << "`\n";
    md << "- Borderline: `" << borderline_count << "`\n";
    md << "- Fail: `" << fail_count << "`\n\n";
    md << "| Node | Sample type | Parameter | Source clip | Pose dist | Root vel seam | Yaw seam | Contact mismatch | Verdict |\n";
    md << "|---|---|---|---|---:|---:|---:|---:|---|\n";
    for (const ArtifactLoopSweepRow& row : rows) {
        md << "| " << row.node_name
           << " | " << row.sample_type
           << " | " << ParameterMd(row.parameter)
           << " | " << (row.source_clip.empty() ? "-" : FormatSegmentLabel(
                    row.source_clip, row.start_frame, row.end_frame))
           << " | " << row.loop.root_normalized_start_end_distance
           << " | " << row.loop.root_velocity_discontinuity
           << " | " << row.loop.root_yaw_discontinuity
           << " | " << row.loop.foot_contact_mismatch
           << " | " << row.verdict << " |\n";
    }
}

std::vector<ArtifactLoopSweepRow> BuildArtifactLoopSweepRows(
    const pmg::BuiltPmgArtifact& artifact) {
    std::vector<ArtifactLoopSweepRow> rows;
    for (int node_index = 0; node_index < artifact.graph.NumNodes(); ++node_index) {
        const pmg::PmgNode& node = artifact.graph.Node(node_index);
        for (const pmg::ExampleMotion& example : node.motion_space.Examples()) {
            ArtifactLoopSweepRow row;
            row.node_name = node.name;
            row.sample_type = "authored_anchor";
            row.parameter = example.parameter;
            row.source_clip = example.segment.source_bvh;
            row.start_frame = example.segment.start_frame;
            row.end_frame = example.segment.end_frame;
            row.loop = pmg::AuditLoop({artifact.skeleton, example.clip});
            row.verdict = LoopSweepVerdict(row.loop);
            rows.push_back(std::move(row));
        }
        for (const pmg::ParameterVector& parameter :
             GeneratedAuditParameters(node.motion_space)) {
            ArtifactLoopSweepRow row;
            row.node_name = node.name;
            row.sample_type = "generated_grid";
            row.parameter = parameter;
            const pmg::MotionClip generated = node.motion_space.GenerateClip(
                parameter, artifact.metadata.frames_per_second);
            row.loop = pmg::AuditLoop({artifact.skeleton, generated});
            row.verdict = LoopSweepVerdict(row.loop);
            rows.push_back(std::move(row));
        }
        for (const pmg::ParameterVector& parameter :
             node.motion_space.ExampleParameters()) {
            ArtifactLoopSweepRow row;
            row.node_name = node.name;
            row.sample_type = "generated_anchor";
            row.parameter = parameter;
            const pmg::MotionClip generated = node.motion_space.GenerateClip(
                parameter, artifact.metadata.frames_per_second);
            row.loop = pmg::AuditLoop({artifact.skeleton, generated});
            row.verdict = LoopSweepVerdict(row.loop);
            rows.push_back(std::move(row));
        }
    }
    return rows;
}

MotionSpaceRegistrationAuditData BuildMotionSpaceRegistrationAudit(
    const pmg::BuiltPmgArtifact& artifact) {
    MotionSpaceRegistrationAuditData data;
    const std::vector<int> contact_joints =
        ResolveRegistrationAuditFootJoints(artifact.skeleton);

    for (int node_index = 0; node_index < artifact.graph.NumNodes(); ++node_index) {
        const pmg::PmgNode& node = artifact.graph.Node(node_index);
        const auto& examples = node.motion_space.Examples();
        const std::size_t node_anchor_offset = data.anchor_rows.size();
        std::vector<std::vector<pmg::ContactInterval>> anchor_contacts(
            examples.size());

        for (std::size_t example_index = 0; example_index < examples.size();
             ++example_index) {
            const pmg::ExampleMotion& example = examples[example_index];
            RegistrationAnchorRow row;
            row.node_name = node.name;
            row.anchor_id = static_cast<int>(example_index);
            row.parameter = example.parameter;
            row.source_bvh_path = example.segment.source_bvh;
            row.start_frame = example.segment.start_frame;
            row.end_frame = example.segment.end_frame;
            row.frame_count = example.clip.NumFrames();
            row.duration_seconds = example.clip.DurationSeconds();
            row.loop = pmg::AuditLoop({artifact.skeleton, example.clip});
            const pmg::RootStartSummary root =
                pmg::SummarizeRootStart(example.clip);
            row.root_displacement = root.final_relative_displacement;
            row.heading_displacement = root.final_relative_heading;
            row.verdict = LoopSweepVerdict(row.loop);
            if (!contact_joints.empty()) {
                const pmg::ContactDetectionSettings settings =
                    pmg::EstimateContactSettings(
                        artifact.skeleton, example.clip, contact_joints);
                anchor_contacts[example_index] = pmg::DetectContacts(
                    artifact.skeleton, example.clip, contact_joints, settings);
                row.start_contact_state = ClipContactStateSummary(
                    anchor_contacts[example_index], contact_joints, 0);
                row.end_contact_state = ClipContactStateSummary(
                    anchor_contacts[example_index], contact_joints,
                    example.clip.NumFrames() - 1);
            } else {
                row.start_contact_state = "n/a";
                row.end_contact_state = "n/a";
            }
            data.anchor_rows.push_back(std::move(row));
        }

        for (std::size_t source_index = 0; source_index < examples.size();
             ++source_index) {
            for (std::size_t target_index = source_index + 1;
                 target_index < examples.size();
                 ++target_index) {
                const pmg::MotionClip& source = examples[source_index].clip;
                const pmg::MotionClip& target = examples[target_index].clip;
                RegistrationPairwiseRow row;
                row.node_name = node.name;
                row.source_anchor_id = static_cast<int>(source_index);
                row.target_anchor_id = static_cast<int>(target_index);
                row.parameter_distance = ParameterDistance(
                    examples[source_index].parameter,
                    examples[target_index].parameter);
                row.start_pose_distance = MeanPoseDistance(
                    artifact.skeleton, source.frames.front(),
                    target.frames.front());
                row.end_pose_distance = MeanPoseDistance(
                    artifact.skeleton, source.frames.back(),
                    target.frames.back());
                row.contact_phase_match =
                    data.anchor_rows[node_anchor_offset + source_index]
                            .start_contact_state ==
                        data.anchor_rows[node_anchor_offset + target_index]
                            .start_contact_state &&
                    data.anchor_rows[node_anchor_offset + source_index]
                            .end_contact_state ==
                        data.anchor_rows[node_anchor_offset + target_index]
                            .end_contact_state;
                row.root_delta_difference =
                    (data.anchor_rows[node_anchor_offset + source_index]
                         .root_displacement -
                     data.anchor_rows[node_anchor_offset + target_index]
                         .root_displacement)
                        .Norm();
                row.heading_delta_difference = std::abs(
                    data.anchor_rows[node_anchor_offset + source_index]
                        .heading_displacement -
                    data.anchor_rows[node_anchor_offset + target_index]
                        .heading_displacement);
                row.duration_frame_match =
                    source.NumFrames() == target.NumFrames() &&
                    std::abs(source.DurationSeconds() - target.DurationSeconds()) <=
                        1.0e-4f;
                data.pair_rows.push_back(std::move(row));
            }
        }

        auto add_generated_row = [&](const std::string& sample_type,
                                     const pmg::ParameterVector& parameter) {
            RegistrationGeneratedRow row;
            row.node_name = node.name;
            row.sample_type = sample_type;
            row.parameter = parameter;
            const pmg::MotionClip generated = node.motion_space.GenerateClip(
                parameter, artifact.metadata.frames_per_second);
            row.loop = pmg::AuditLoop({artifact.skeleton, generated});
            row.verdict = LoopSweepVerdict(row.loop);
            const std::vector<float> weights =
                node.motion_space.ComputeLocalBlendWeights(parameter);
            for (std::size_t anchor_index = 0; anchor_index < weights.size();
                 ++anchor_index) {
                if (weights[anchor_index] <= 1.0e-4f) {
                    continue;
                }
                const RegistrationAnchorRow& anchor =
                    data.anchor_rows[node_anchor_offset + anchor_index];
                row.contributing_anchor_ids.push_back(
                    static_cast<int>(anchor_index));
                row.blend_weights.push_back(weights[anchor_index]);
                row.contributing_source_bvh_paths.push_back(anchor.source_bvh_path);
                row.contributing_anchor_verdicts.push_back(anchor.verdict);
                row.contributing_anchor_cycle_scores.push_back(
                    anchor.loop.cycle_score);
                row.contributing_anchor_contact_mismatch.push_back(
                    anchor.loop.foot_contact_mismatch);
                row.contributing_anchor_start_contact_state.push_back(
                    anchor.start_contact_state);
                row.contributing_anchor_end_contact_state.push_back(
                    anchor.end_contact_state);
                row.contributing_anchor_root_deltas.push_back(
                    anchor.root_displacement);
                row.contributing_anchor_heading_deltas.push_back(
                    anchor.heading_displacement);
            }
            data.generated_rows.push_back(std::move(row));
        };

        for (const pmg::ParameterVector& parameter :
             GeneratedAuditParameters(node.motion_space)) {
            add_generated_row("generated_grid", parameter);
        }
        for (const pmg::ParameterVector& parameter :
             node.motion_space.ExampleParameters()) {
            add_generated_row("generated_anchor", parameter);
        }
    }

    int bad_anchor_count = 0;
    bool has_failed_generated_anchor = false;
    int failed_generated_count = 0;
    int failed_generated_with_bad_anchor = 0;
    int failed_generated_with_contact_mismatch = 0;
    int failed_generated_with_large_root_blend = 0;
    int failed_generated_with_diffuse_weights = 0;
    for (const RegistrationAnchorRow& row : data.anchor_rows) {
        if (row.verdict == "fail") {
            ++bad_anchor_count;
        }
    }
    for (const RegistrationGeneratedRow& row : data.generated_rows) {
        has_failed_generated_anchor |=
            row.sample_type == "generated_anchor" && row.verdict == "fail";
        if (row.verdict != "fail") {
            continue;
        }
        ++failed_generated_count;
        bool has_bad_anchor = false;
        bool has_contact_mismatch = false;
        float max_weight = 0.0f;
        float max_root_delta_difference = 0.0f;
        for (std::size_t index = 0; index < row.contributing_anchor_ids.size();
             ++index) {
            max_weight = std::max(max_weight, row.blend_weights[index]);
            has_bad_anchor |= row.contributing_anchor_verdicts[index] == "fail";
            has_contact_mismatch |=
                row.contributing_anchor_start_contact_state[index] !=
                    row.contributing_anchor_end_contact_state[index] ||
                row.contributing_anchor_contact_mismatch[index] > 0;
            for (std::size_t other = index + 1;
                 other < row.contributing_anchor_ids.size(); ++other) {
                max_root_delta_difference = std::max(
                    max_root_delta_difference,
                    (row.contributing_anchor_root_deltas[index] -
                     row.contributing_anchor_root_deltas[other])
                        .Norm());
            }
        }
        failed_generated_with_bad_anchor += has_bad_anchor ? 1 : 0;
        failed_generated_with_contact_mismatch += has_contact_mismatch ? 1 : 0;
        failed_generated_with_large_root_blend +=
            max_root_delta_difference > 10.0f ? 1 : 0;
        failed_generated_with_diffuse_weights +=
            (row.blend_weights.size() >= 3 && max_weight < 0.6f) ? 1 : 0;
    }

    if (bad_anchor_count > 0 && has_failed_generated_anchor) {
        data.root_cause = RegistrationRootCause::kBadSourceAnchorLoop;
    } else if (bad_anchor_count > 0 &&
               failed_generated_count > 0 &&
               failed_generated_with_bad_anchor == failed_generated_count) {
        data.root_cause = RegistrationRootCause::kBadSourceAnchorLoop;
    } else if (failed_generated_with_contact_mismatch > 0 &&
               failed_generated_with_large_root_blend > 0) {
        data.root_cause = RegistrationRootCause::kMixedFailure;
    } else if (failed_generated_with_contact_mismatch > 0) {
        data.root_cause = RegistrationRootCause::kAnchorPhaseMismatch;
    } else if (failed_generated_with_large_root_blend > 0) {
        data.root_cause = RegistrationRootCause::kRootTrajectoryBlendFailure;
    } else if (failed_generated_with_diffuse_weights > 0) {
        data.root_cause = RegistrationRootCause::kInterpolationWeightFailure;
    } else {
        data.root_cause = RegistrationRootCause::kUnknown;
    }

    return data;
}

void WriteMotionSpaceRegistrationAnchorCsv(
    const std::filesystem::path& path,
    const MotionSpaceRegistrationAuditData& data) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "audit-motion-space-registration: cannot write anchor_registration.csv");
    }
    csv << "node_name,anchor_id,parameter,source_bvh_path,start_frame,end_frame,frame_count,duration_seconds,"
           "cycle_score,root_normalized_start_end_distance,root_velocity_seam,"
           "yaw_seam,contact_mismatch,start_contact_state,end_contact_state,"
           "root_displacement,heading_displacement,verdict\n";
    for (const RegistrationAnchorRow& row : data.anchor_rows) {
        csv << EscapeCsvCell(row.node_name) << ',' << row.anchor_id << ','
            << ParameterCsv(row.parameter) << ','
            << EscapeCsvCell(row.source_bvh_path) << ',' << row.start_frame << ','
            << row.end_frame << ',' << row.frame_count << ','
            << row.duration_seconds << ',' << row.loop.cycle_score << ','
            << row.loop.root_normalized_start_end_distance << ','
            << row.loop.root_velocity_discontinuity << ','
            << row.loop.root_yaw_discontinuity << ','
            << row.loop.foot_contact_mismatch << ','
            << EscapeCsvCell(row.start_contact_state) << ','
            << EscapeCsvCell(row.end_contact_state) << ','
            << Vec3Csv(row.root_displacement) << ','
            << row.heading_displacement << ',' << row.verdict << "\n";
    }
}

void WriteMotionSpaceRegistrationAnchorMarkdown(
    const std::filesystem::path& path,
    const MotionSpaceRegistrationAuditData& data) {
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "audit-motion-space-registration: cannot write anchor_registration.md");
    }
    const int pass_count = static_cast<int>(std::count_if(
        data.anchor_rows.begin(), data.anchor_rows.end(),
        [](const RegistrationAnchorRow& row) { return row.verdict == "pass"; }));
    const int borderline_count = static_cast<int>(std::count_if(
        data.anchor_rows.begin(), data.anchor_rows.end(),
        [](const RegistrationAnchorRow& row) {
            return row.verdict == "borderline";
        }));
    const int fail_count = static_cast<int>(std::count_if(
        data.anchor_rows.begin(), data.anchor_rows.end(),
        [](const RegistrationAnchorRow& row) { return row.verdict == "fail"; }));
    md << "# Anchor Registration Audit\n\n";
    md << "- Anchors: `" << data.anchor_rows.size() << "`\n";
    md << "- Pass: `" << pass_count << "`\n";
    md << "- Borderline: `" << borderline_count << "`\n";
    md << "- Fail: `" << fail_count << "`\n\n";
    md << "| Anchor | Parameter | Source BVH | Cycle score | Root vel seam | Yaw seam | Contact mismatch | Start contact | End contact | Root delta | Heading delta | Verdict |\n";
    md << "|---:|---|---|---:|---:|---:|---:|---|---|---|---:|---|\n";
    for (const RegistrationAnchorRow& row : data.anchor_rows) {
        md << "| " << row.anchor_id << " | " << ParameterMd(row.parameter)
           << " | " << FormatSegmentLabel(
                    row.source_bvh_path, row.start_frame, row.end_frame)
           << " | " << row.loop.cycle_score
           << " | " << row.loop.root_velocity_discontinuity << " | "
           << row.loop.root_yaw_discontinuity << " | "
           << row.loop.foot_contact_mismatch << " | "
           << row.start_contact_state << " | " << row.end_contact_state
           << " | [" << row.root_displacement.x << ", "
           << row.root_displacement.z << "] | " << row.heading_displacement
           << " | " << row.verdict << " |\n";
    }
}

void WriteMotionSpaceRegistrationPairwiseCsv(
    const std::filesystem::path& path,
    const MotionSpaceRegistrationAuditData& data) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "audit-motion-space-registration: cannot write anchor_pairwise_consistency.csv");
    }
    csv << "node_name,source_anchor_id,target_anchor_id,parameter_distance,"
           "start_pose_distance,end_pose_distance,contact_phase_match,"
           "root_delta_difference,heading_delta_difference,duration_frame_match\n";
    for (const RegistrationPairwiseRow& row : data.pair_rows) {
        csv << EscapeCsvCell(row.node_name) << ',' << row.source_anchor_id << ','
            << row.target_anchor_id << ',' << row.parameter_distance << ','
            << row.start_pose_distance << ',' << row.end_pose_distance << ','
            << (row.contact_phase_match ? "true" : "false") << ','
            << row.root_delta_difference << ','
            << row.heading_delta_difference << ','
            << (row.duration_frame_match ? "true" : "false") << "\n";
    }
}

void WriteMotionSpaceRegistrationGeneratedCsv(
    const std::filesystem::path& path,
    const MotionSpaceRegistrationAuditData& data) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "audit-motion-space-registration: cannot write generated_loop_provenance.csv");
    }
    csv << "node_name,sample_type,parameter,loop_status,cycle_score,root_velocity_seam,"
           "yaw_seam,contact_mismatch,contributing_anchor_ids,blend_weights,"
           "contributing_source_bvh_paths,contributing_anchor_verdicts,"
           "contributing_anchor_cycle_scores,contributing_anchor_contact_mismatch,"
           "contributing_anchor_start_contact_state,"
           "contributing_anchor_end_contact_state,contributing_anchor_root_deltas,"
           "contributing_anchor_heading_deltas\n";
    for (const RegistrationGeneratedRow& row : data.generated_rows) {
        csv << EscapeCsvCell(row.node_name) << ','
            << EscapeCsvCell(row.sample_type) << ','
            << ParameterCsv(row.parameter) << ',' << row.verdict << ','
            << row.loop.cycle_score << ','
            << row.loop.root_velocity_discontinuity << ','
            << row.loop.root_yaw_discontinuity << ','
            << row.loop.foot_contact_mismatch << ','
            << EscapeCsvCell(JoinIntList(row.contributing_anchor_ids)) << ','
            << EscapeCsvCell(JoinFloatList(row.blend_weights)) << ','
            << EscapeCsvCell(JoinStringList(row.contributing_source_bvh_paths))
            << ','
            << EscapeCsvCell(JoinStringList(row.contributing_anchor_verdicts))
            << ','
            << EscapeCsvCell(JoinFloatList(
                   row.contributing_anchor_cycle_scores))
            << ','
            << EscapeCsvCell(JoinIntList(
                   row.contributing_anchor_contact_mismatch))
            << ','
            << EscapeCsvCell(JoinStringList(
                   row.contributing_anchor_start_contact_state))
            << ','
            << EscapeCsvCell(JoinStringList(
                   row.contributing_anchor_end_contact_state))
            << ','
            << EscapeCsvCell(JoinVec3List(
                   row.contributing_anchor_root_deltas))
            << ','
            << EscapeCsvCell(JoinFloatList(
                   row.contributing_anchor_heading_deltas))
            << "\n";
    }
}

void WriteMotionSpaceRegistrationGeneratedMarkdown(
    const std::filesystem::path& path,
    const MotionSpaceRegistrationAuditData& data) {
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "audit-motion-space-registration: cannot write generated_loop_provenance.md");
    }
    const int pass_count = static_cast<int>(std::count_if(
        data.generated_rows.begin(), data.generated_rows.end(),
        [](const RegistrationGeneratedRow& row) { return row.verdict == "pass"; }));
    const int borderline_count = static_cast<int>(std::count_if(
        data.generated_rows.begin(), data.generated_rows.end(),
        [](const RegistrationGeneratedRow& row) {
            return row.verdict == "borderline";
        }));
    const int fail_count = static_cast<int>(std::count_if(
        data.generated_rows.begin(), data.generated_rows.end(),
        [](const RegistrationGeneratedRow& row) { return row.verdict == "fail"; }));
    md << "# Generated Loop Provenance\n\n";
    md << "- Samples: `" << data.generated_rows.size() << "`\n";
    md << "- Pass: `" << pass_count << "`\n";
    md << "- Borderline: `" << borderline_count << "`\n";
    md << "- Fail: `" << fail_count << "`\n";
    md << "- Root cause label: `" << RegistrationRootCauseName(data.root_cause)
       << "`\n\n";
    md << "| Sample type | Parameter | Verdict | Cycle score | Root vel seam | Yaw seam | Contact mismatch | Anchors | Weights | Source BVHs |\n";
    md << "|---|---|---|---:|---:|---:|---:|---|---|---|\n";
    for (const RegistrationGeneratedRow& row : data.generated_rows) {
        md << "| " << row.sample_type << " | " << ParameterMd(row.parameter)
           << " | " << row.verdict << " | " << row.loop.cycle_score << " | "
           << row.loop.root_velocity_discontinuity << " | "
           << row.loop.root_yaw_discontinuity << " | "
           << row.loop.foot_contact_mismatch << " | "
           << JoinIntList(row.contributing_anchor_ids) << " | "
           << JoinFloatList(row.blend_weights) << " | "
           << JoinStringList(row.contributing_source_bvh_paths) << " |\n";
    }
}

MotionSpaceRegistrationAuditOptions ParseMotionSpaceRegistrationAuditOptions(
    int argc,
    char** argv) {
    if (argc < 5) {
        throw std::runtime_error(
            "usage: pmg_cli audit-motion-space-registration artifact.pmg --out outputs/diagnostics/walk2d_registration");
    }
    MotionSpaceRegistrationAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            return argv[++index];
        };
        if (option == "--out") {
            options.out_dir = require_value("--out");
        } else {
            throw std::runtime_error(
                "audit-motion-space-registration: unknown option '" + option +
                "'");
        }
    }
    if (options.out_dir.empty()) {
        throw std::runtime_error(
            "audit-motion-space-registration requires --out");
    }
    return options;
}

int MotionSpaceRegistrationAuditCommand(
    const MotionSpaceRegistrationAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact = pmg::LoadPmgArtifactText(options.pmg_path);
    const MotionSpaceRegistrationAuditData data =
        BuildMotionSpaceRegistrationAudit(artifact);
    std::filesystem::create_directories(options.out_dir);
    WriteMotionSpaceRegistrationAnchorCsv(
        options.out_dir / "anchor_registration.csv", data);
    WriteMotionSpaceRegistrationAnchorMarkdown(
        options.out_dir / "anchor_registration.md", data);
    WriteMotionSpaceRegistrationPairwiseCsv(
        options.out_dir / "anchor_pairwise_consistency.csv", data);
    WriteMotionSpaceRegistrationGeneratedCsv(
        options.out_dir / "generated_loop_provenance.csv", data);
    WriteMotionSpaceRegistrationGeneratedMarkdown(
        options.out_dir / "generated_loop_provenance.md", data);
    std::cout << "registration_anchor_csv="
              << (options.out_dir / "anchor_registration.csv").string() << "\n";
    std::cout << "registration_anchor_md="
              << (options.out_dir / "anchor_registration.md").string() << "\n";
    std::cout << "registration_pairwise_csv="
              << (options.out_dir / "anchor_pairwise_consistency.csv").string()
              << "\n";
    std::cout << "registration_generated_csv="
              << (options.out_dir / "generated_loop_provenance.csv").string()
              << "\n";
    std::cout << "registration_generated_md="
              << (options.out_dir / "generated_loop_provenance.md").string()
              << "\n";
    std::cout << "registration_root_cause="
              << RegistrationRootCauseName(data.root_cause) << "\n";
    return 0;
}

struct ReachableRegionAuditRow {
    pmg::ParameterVector source_parameter;
    pmg::ParameterAabb target_box;
    std::vector<pmg::ParameterVector> accepted_target_parameters;
    int accepted_target_count = 0;
    float support_coverage = 0.0f;
    int good_count = 0;
    int neutral_count = 0;
    int bad_count = 0;
    float worst_accepted_distance = 0.0f;
    float worst_root_jump = 0.0f;
    float worst_heading_jump = 0.0f;
    float worst_velocity_jump = 0.0f;
    bool empty_region = false;
    std::string conclusion;
};

struct ReachableRegionAuditData {
    std::string source_node;
    std::string target_node;
    pmg::PmgBuilderConfig config;
    std::vector<pmg::ParameterVector> target_samples;
    std::vector<ReachableRegionAuditRow> rows;
    std::string conclusion;
};

void AppendUniqueParameter(
    std::vector<pmg::ParameterVector>& parameters,
    const pmg::ParameterVector& candidate) {
    for (const pmg::ParameterVector& existing : parameters) {
        if (existing.size() != candidate.size()) {
            continue;
        }
        bool same = true;
        for (std::size_t axis = 0; axis < existing.size(); ++axis) {
            if (std::abs(existing[axis] - candidate[axis]) > 1.0e-5f) {
                same = false;
                break;
            }
        }
        if (same) {
            return;
        }
    }
    parameters.push_back(candidate);
}

std::vector<pmg::ParameterVector> BuildAuditParameterSamples(
    const pmg::ParametricMotionSpace& space,
    int samples_per_axis) {
    if (samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-reachable-region: --samples must be at least 2");
    }

    std::vector<pmg::ParameterVector> samples;
    for (const pmg::ParameterVector& parameter : space.ExampleParameters()) {
        AppendUniqueParameter(samples, parameter);
    }

    if (space.HasExplicitParameterSupport()) {
        const pmg::ParameterSupport& support = *space.ExplicitSupport();
        for (const pmg::ParameterVector& vertex : support.Vertices()) {
            AppendUniqueParameter(samples, vertex);
        }
        if (support.GetType() == pmg::ParameterSupport::Type::kTriangulated2D) {
            std::set<std::pair<int, int>> edges;
            for (const auto& triangle : support.Triangles()) {
                pmg::ParameterVector centroid(2, 0.0f);
                for (int vertex : triangle) {
                    centroid[0] += support.Vertices()[vertex][0] / 3.0f;
                    centroid[1] += support.Vertices()[vertex][1] / 3.0f;
                }
                AppendUniqueParameter(samples, centroid);
                for (int edge = 0; edge < 3; ++edge) {
                    int a = triangle[edge];
                    int b = triangle[(edge + 1) % 3];
                    if (a > b) {
                        std::swap(a, b);
                    }
                    if (!edges.insert({a, b}).second) {
                        continue;
                    }
                    AppendUniqueParameter(
                        samples,
                        {(support.Vertices()[a][0] + support.Vertices()[b][0]) * 0.5f,
                         (support.Vertices()[a][1] + support.Vertices()[b][1]) * 0.5f});
                }
            }
        }
    }

    const pmg::ParameterDomain domain = space.Domain();
    const pmg::ParameterAabb& bounds = domain.Bounds();
    if (domain.Dimension() == 1) {
        for (int index = 0; index < samples_per_axis; ++index) {
            const float t =
                static_cast<float>(index) /
                static_cast<float>(samples_per_axis - 1);
            const pmg::ParameterVector parameter = {
                bounds.min_corner[0] +
                t * (bounds.max_corner[0] - bounds.min_corner[0])};
            if (!space.HasExplicitParameterSupport() ||
                space.ExplicitSupport()->Contains(parameter)) {
                AppendUniqueParameter(samples, parameter);
            }
        }
    } else if (domain.Dimension() == 2) {
        for (int y_index = 0; y_index < samples_per_axis; ++y_index) {
            const float ty =
                static_cast<float>(y_index) /
                static_cast<float>(samples_per_axis - 1);
            const float y = bounds.min_corner[1] +
                            ty * (bounds.max_corner[1] - bounds.min_corner[1]);
            for (int x_index = 0; x_index < samples_per_axis; ++x_index) {
                const float tx =
                    static_cast<float>(x_index) /
                    static_cast<float>(samples_per_axis - 1);
                const float x =
                    bounds.min_corner[0] +
                    tx * (bounds.max_corner[0] - bounds.min_corner[0]);
                const pmg::ParameterVector parameter = {x, y};
                if (!space.HasExplicitParameterSupport() ||
                    space.ExplicitSupport()->Contains(parameter)) {
                    AppendUniqueParameter(samples, parameter);
                }
            }
        }
    }

    return samples;
}

const pmg::EdgeBuildMetadata* FindEdgeBuildMetadata(
    const pmg::BuiltPmgArtifact& artifact,
    const std::string& source_node,
    const std::string& target_node) {
    for (const pmg::EdgeBuildMetadata& edge : artifact.metadata.edge_builds) {
        if (edge.source_node == source_node && edge.target_node == target_node) {
            return &edge;
        }
    }
    return nullptr;
}

pmg::OptimalTransition EvaluateConfiguredTransition(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::PmgBuilderConfig& config) {
    if (config.transition_metric_type ==
        pmg::TransitionMetricType::kDynamicsWindow) {
        return pmg::MotionDistance::FindOptimalDynamicsTransitionForConvention(
            skeleton, source_clip, target_clip, config.distance_grid,
            config.transition_metric, config.transition_convention);
    }
    return pmg::MotionDistance::FindOptimalTransitionForConvention(
        skeleton, source_clip, target_clip, config.distance_grid,
        config.transition_convention);
}

float RootJumpAtTransition(
    const pmg::Pose& source_pose,
    const pmg::Pose& aligned_target_pose) {
    return HorizontalDistance(
        aligned_target_pose.root_position - source_pose.root_position);
}

float HeadingJumpAtTransition(
    const pmg::Pose& source_pose,
    const pmg::Pose& aligned_target_pose) {
    return std::abs(
        pmg::WrapAngleRadians(
            pmg::RootHeadingYaw(aligned_target_pose) -
            pmg::RootHeadingYaw(source_pose)));
}

float VelocityJumpAtTransition(
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::RigidTransform2D& alignment,
    int source_frame,
    int target_frame) {
    if (source_frame <= 0 || target_frame <= 0) {
        return 0.0f;
    }

    const pmg::Vec3 source_step =
        source_clip.frames[static_cast<std::size_t>(source_frame)].root_position -
        source_clip.frames[static_cast<std::size_t>(source_frame - 1)].root_position;
    const pmg::Vec3 target_step =
        alignment.ApplyPoint(
            target_clip.frames[static_cast<std::size_t>(target_frame)].root_position) -
        alignment.ApplyPoint(
            target_clip.frames[static_cast<std::size_t>(target_frame - 1)].root_position);
    const pmg::Vec3 delta =
        (target_step - source_step) * source_clip.frames_per_second;
    return HorizontalLength(delta);
}

int ResolveNodeIndex(
    const pmg::ParametricMotionGraph& graph,
    const std::string& node_name) {
    for (int node_index = 0; node_index < graph.NumNodes(); ++node_index) {
        if (graph.Node(node_index).name == node_name) {
            return node_index;
        }
    }
    return -1;
}

int ResolveEdgeIndex(
    const pmg::BuiltPmgArtifact& artifact,
    const ReachableRegionAuditOptions& options,
    std::string& source_node,
    std::string& target_node) {
    if (!options.source_node.empty() || !options.target_node.empty()) {
        if (options.source_node.empty() || options.target_node.empty()) {
            throw std::runtime_error(
                "--audit-reachable-region requires both --source-node and "
                "--target-node");
        }
        const int source_index =
            ResolveNodeIndex(artifact.graph, options.source_node);
        const int target_index =
            ResolveNodeIndex(artifact.graph, options.target_node);
        if (source_index < 0 || target_index < 0) {
            throw std::runtime_error(
                "--audit-reachable-region: unknown source or target node");
        }
        for (int edge_index = 0; edge_index < artifact.graph.NumEdges();
             ++edge_index) {
            const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
            if (edge.source_node == source_index &&
                edge.target_node == target_index) {
                source_node = options.source_node;
                target_node = options.target_node;
                return edge_index;
            }
        }
        throw std::runtime_error(
            "--audit-reachable-region: requested edge not found");
    }

    if (artifact.graph.NumEdges() != 1) {
        throw std::runtime_error(
            "--audit-reachable-region requires --source-node and --target-node "
            "when artifact has multiple edges");
    }

    const pmg::PmgEdge& edge = artifact.graph.Edge(0);
    source_node = artifact.graph.Node(edge.source_node).name;
    target_node = artifact.graph.Node(edge.target_node).name;
    return 0;
}

ReachableRegionAuditData BuildReachableRegionAudit(
    const pmg::BuiltPmgArtifact& artifact,
    const ReachableRegionAuditOptions& options) {
    ReachableRegionAuditData data;
    int edge_index = -1;
    edge_index = ResolveEdgeIndex(
        artifact, options, data.source_node, data.target_node);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* edge_metadata =
        FindEdgeBuildMetadata(artifact, data.source_node, data.target_node);
    if (edge_metadata == nullptr) {
        throw std::runtime_error(
            "--audit-reachable-region: missing edge build metadata");
    }
    data.config = edge_metadata->config;
    data.target_samples = BuildAuditParameterSamples(
        target_node.motion_space, options.samples_per_axis);
    for (const pmg::TransitionSample& sample : edge.samples) {
        for (const pmg::TargetTransitionPhaseSample& phase_sample :
             sample.target_phase_samples) {
            AppendUniqueParameter(data.target_samples, phase_sample.target_parameter);
        }
    }

    std::vector<int> accepted_density(
        data.target_samples.size(), 0);
    std::vector<float> worst_target_distance(
        data.target_samples.size(), 0.0f);

    constexpr float kCoverageShrinkWarnRatio = 0.5f;
    float max_support_coverage = 0.0f;
    bool any_empty = false;
    bool any_bad_accepted = false;

    for (std::size_t source_index = 0; source_index < edge.samples.size();
         ++source_index) {
        const pmg::TransitionSample& source_sample = edge.samples[source_index];
        ReachableRegionAuditRow row;
        row.source_parameter = source_sample.source_parameter;

        const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
            source_sample.source_parameter,
            artifact.metadata.frames_per_second);

        for (std::size_t target_index = 0; target_index < data.target_samples.size();
             ++target_index) {
            const pmg::ParameterVector& target_parameter =
                data.target_samples[target_index];
            const std::optional<pmg::InterpolatedTransition> interpolated =
                edge.LookupInterpolated(
                    source_sample.source_parameter, target_parameter);
            if (!interpolated.has_value()) {
                continue;
            }

            row.target_box = interpolated->target_parameter_box;
            const pmg::MotionClip target_clip =
                target_node.motion_space.GenerateClip(
                    target_parameter, artifact.metadata.frames_per_second);
            const pmg::OptimalTransition transition =
                EvaluateConfiguredTransition(
                    artifact.skeleton, source_clip, target_clip, data.config);
            if (transition.distance <= data.config.good_transition_threshold) {
                ++row.good_count;
            } else if (transition.distance >= data.config.bad_transition_threshold) {
                ++row.bad_count;
            } else {
                ++row.neutral_count;
            }

            if (!interpolated->target_parameter_box.Contains(target_parameter)) {
                continue;
            }

            ++row.accepted_target_count;
            AppendUniqueParameter(row.accepted_target_parameters, target_parameter);
            ++accepted_density[target_index];
            worst_target_distance[target_index] = std::max(
                worst_target_distance[target_index], transition.distance);
            row.worst_accepted_distance = std::max(
                row.worst_accepted_distance, transition.distance);
            const pmg::Pose source_pose =
                source_clip.frames[static_cast<std::size_t>(transition.source_frame)];
            const pmg::Pose aligned_target_pose = transition.alignment.Apply(
                target_clip.frames[static_cast<std::size_t>(transition.target_frame)]);
            row.worst_root_jump = std::max(
                row.worst_root_jump,
                RootJumpAtTransition(source_pose, aligned_target_pose));
            row.worst_heading_jump = std::max(
                row.worst_heading_jump,
                HeadingJumpAtTransition(source_pose, aligned_target_pose));
            row.worst_velocity_jump = std::max(
                row.worst_velocity_jump,
                VelocityJumpAtTransition(
                    source_clip, target_clip, transition.alignment,
                    transition.source_frame, transition.target_frame));
        }

        if (!data.target_samples.empty()) {
            row.support_coverage =
                static_cast<float>(row.accepted_target_count) /
                static_cast<float>(data.target_samples.size());
        }
        row.empty_region = row.accepted_target_count == 0;
        max_support_coverage = std::max(max_support_coverage, row.support_coverage);
        any_empty = any_empty || row.empty_region;
        any_bad_accepted =
            any_bad_accepted ||
            row.worst_accepted_distance >= data.config.bad_transition_threshold;
        data.rows.push_back(std::move(row));
    }

    data.conclusion = "PASS_REACHABLE_REGION_STABLE";
    for (ReachableRegionAuditRow& row : data.rows) {
        if (row.empty_region) {
            row.conclusion = "FAIL_EMPTY_REGION_FOR_VALID_SOURCE";
            continue;
        }
        if (row.worst_accepted_distance >= data.config.bad_transition_threshold) {
            row.conclusion = "FAIL_THRESHOLD_TOO_LOOSE";
            continue;
        }
        if (max_support_coverage > 0.0f &&
            row.support_coverage < max_support_coverage * kCoverageShrinkWarnRatio) {
            row.conclusion = "WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE";
            continue;
        }
        row.conclusion = "PASS_REACHABLE_REGION_STABLE";
    }

    if (any_empty) {
        data.conclusion = "FAIL_EMPTY_REGION_FOR_VALID_SOURCE";
    } else if (any_bad_accepted) {
        data.conclusion = "FAIL_THRESHOLD_TOO_LOOSE";
    } else {
        for (const ReachableRegionAuditRow& row : data.rows) {
            if (row.conclusion == "WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE") {
                data.conclusion = "WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE";
                break;
            }
        }
    }

    std::filesystem::create_directories(options.output_dir);
    {
        std::ofstream density(options.output_dir + "/accepted_density.csv");
        if (!density) {
            throw std::runtime_error(
                "--audit-reachable-region: cannot write accepted density map");
        }
        density << "target_parameter,accepted_source_count,worst_accepted_distance\n";
        for (std::size_t target_index = 0; target_index < data.target_samples.size();
             ++target_index) {
            density << ParameterCsv(data.target_samples[target_index]) << ','
                    << accepted_density[target_index] << ','
                    << worst_target_distance[target_index] << "\n";
        }
    }
    for (std::size_t source_index = 0; source_index < data.rows.size();
         ++source_index) {
        std::ostringstream path;
        path << options.output_dir << "/source_"
             << std::setw(2) << std::setfill('0') << source_index << ".csv";
        std::ofstream csv(path.str());
        if (!csv) {
            throw std::runtime_error(
                "--audit-reachable-region: cannot write per-source map");
        }
        csv << "source_parameter,target_parameter,accepted,target_box_min,target_box_max,"
               "transition_distance,transition_class,root_jump,heading_jump,"
               "velocity_jump\n";
        const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
            data.rows[source_index].source_parameter,
            artifact.metadata.frames_per_second);
        for (const pmg::ParameterVector& target_parameter : data.target_samples) {
            const std::optional<pmg::InterpolatedTransition> interpolated =
                edge.LookupInterpolated(
                    data.rows[source_index].source_parameter, target_parameter);
            if (!interpolated.has_value()) {
                continue;
            }
            const pmg::MotionClip target_clip =
                target_node.motion_space.GenerateClip(
                    target_parameter, artifact.metadata.frames_per_second);
            const pmg::OptimalTransition transition =
                EvaluateConfiguredTransition(
                    artifact.skeleton, source_clip, target_clip, data.config);
            const bool accepted =
                interpolated->target_parameter_box.Contains(target_parameter);
            const pmg::Pose source_pose =
                source_clip.frames[static_cast<std::size_t>(transition.source_frame)];
            const pmg::Pose aligned_target_pose = transition.alignment.Apply(
                target_clip.frames[static_cast<std::size_t>(transition.target_frame)]);
            csv << ParameterCsv(data.rows[source_index].source_parameter) << ','
                << ParameterCsv(target_parameter) << ','
                << (accepted ? "true" : "false") << ','
                << ParameterCsv(interpolated->target_parameter_box.min_corner) << ','
                << ParameterCsv(interpolated->target_parameter_box.max_corner) << ','
                << transition.distance << ','
                << PoseSeamClass(transition.distance, data.config) << ','
                << RootJumpAtTransition(source_pose, aligned_target_pose) << ','
                << HeadingJumpAtTransition(source_pose, aligned_target_pose) << ','
                << VelocityJumpAtTransition(
                       source_clip, target_clip, transition.alignment,
                       transition.source_frame, transition.target_frame)
                << "\n";
        }
    }
    return data;
}

struct TransitionMontageRow {
    std::string category;
    int category_rank = 0;
    std::string montage_id;
    std::string replay_id;
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector requested_target_parameter;
    pmg::ParameterVector effective_target_parameter;
    float source_phase = 0.0f;
    float target_phase = 0.0f;
    int source_frame = -1;
    int target_frame = -1;
    float transition_distance = 0.0f;
    std::string transition_class;
    float root_jump = 0.0f;
    float heading_jump = 0.0f;
    float velocity_jump = 0.0f;
    int blend_frames = 0;
    std::string notes;
};

struct TransitionMontageAuditData {
    std::string source_node;
    std::string target_node;
    pmg::PmgBuilderConfig config;
    std::string manifest_csv_path;
    std::vector<TransitionMontageRow> rows;
    int unique_transition_count = 0;
    int accepted_count = 0;
    int rejected_count = 0;
    int rejected_jog_walk_count = 0;
    std::string conclusion;
};

using TransitionAcceptanceConsistencyRow =
    pmg::TransitionAcceptanceAuditRow;
using TransitionAcceptanceConsistencyAuditData =
    pmg::TransitionAcceptanceAuditSummary;

constexpr float kTransitionMontageAnchorTolerance = 1.0e-5f;
constexpr float kTransitionMontageProjectedNoteTolerance = 1.0e-4f;
constexpr int kWorstAcceptedSamples = 8;
constexpr int kNearThresholdSamples = 8;
constexpr int kRejectedJogWalkSamples = 8;
constexpr int kAnchorToAnchorSamples = 12;
constexpr int kOutsideRequestSamples = 8;
constexpr int kShrinkageSamples = 8;

bool SameParameterWithin(
    const pmg::ParameterVector& left,
    const pmg::ParameterVector& right,
    float tolerance) {
    if (left.size() != right.size()) {
        return false;
    }
    for (std::size_t axis = 0; axis < left.size(); ++axis) {
        if (std::abs(left[axis] - right[axis]) > tolerance) {
            return false;
        }
    }
    return true;
}

std::string NearestAnchorLabel(
    const pmg::ParameterSupport& support,
    const pmg::ParameterVector& parameter) {
    int best_index = -1;
    float best_distance = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0; index < support.Vertices().size(); ++index) {
        const float distance =
            pmg::SquaredDistance(parameter, support.Vertices()[index]);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(index);
        }
    }
    return best_index >= 0 ? "anchor_" + std::to_string(best_index)
                           : "anchor_unknown";
}

bool IsAnchorParameter(
    const pmg::ParameterSupport& support,
    const pmg::ParameterVector& parameter) {
    for (const pmg::ParameterVector& vertex : support.Vertices()) {
        if (SameParameterWithin(
                parameter, vertex, kTransitionMontageAnchorTolerance)) {
            return true;
        }
    }
    return false;
}

float ParameterAxisOrZero(const pmg::ParameterVector& parameter, int axis) {
    return axis >= 0 && axis < static_cast<int>(parameter.size())
               ? parameter[static_cast<std::size_t>(axis)]
               : 0.0f;
}

bool IsJogWalkPair(
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& target_parameter) {
    const float source_speed_axis = ParameterAxisOrZero(source_parameter, 1);
    const float target_speed_axis = ParameterAxisOrZero(target_parameter, 1);
    return std::max(source_speed_axis, target_speed_axis) >= 0.75f &&
           std::min(source_speed_axis, target_speed_axis) <= 0.25f;
}

struct TransitionAuditCandidate {
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector requested_target_parameter;
    pmg::ParameterVector effective_target_parameter;
    pmg::ParameterAabb interpolated_target_box;
    pmg::OptimalTransition transition;
    bool accepted = false;
    bool source_is_anchor = false;
    bool requested_target_is_anchor = false;
    bool requested_target_projected = false;
    bool effective_target_projected = false;
    bool jog_walk_pair = false;
    bool source_coverage_available = false;
    float source_coverage = 0.0f;
    std::string transition_class;
    std::string notes;
};

TransitionAuditCandidate BuildTransitionAuditCandidate(
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgEdge& edge,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const ReachableRegionAuditData& reachable_data,
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& requested_target_parameter,
    const pmg::ParameterSupport* source_support,
    const pmg::ParameterSupport* target_support) {
    const std::optional<pmg::InterpolatedTransition> interpolated =
        edge.LookupInterpolated(source_parameter, requested_target_parameter);
    if (!interpolated.has_value()) {
        throw std::runtime_error(
            "transition montage audit: edge lookup returned no transition");
    }

    TransitionAuditCandidate candidate;
    candidate.source_parameter = source_parameter;
    candidate.requested_target_parameter = requested_target_parameter;
    candidate.effective_target_parameter =
        interpolated->target_parameter_box.Clamp(requested_target_parameter);
    candidate.interpolated_target_box = interpolated->target_parameter_box;
    if (target_support != nullptr) {
        candidate.effective_target_parameter =
            target_support->ProjectInside(
                requested_target_parameter,
                interpolated->target_parameter_box);
    }
    candidate.accepted =
        interpolated->target_parameter_box.Contains(requested_target_parameter);
    candidate.source_is_anchor =
        source_support != nullptr &&
        IsAnchorParameter(*source_support, source_parameter);
    candidate.requested_target_is_anchor =
        target_support != nullptr &&
        IsAnchorParameter(*target_support, requested_target_parameter);
    candidate.requested_target_projected =
        target_support != nullptr &&
        !target_support->Contains(requested_target_parameter);
    candidate.effective_target_projected =
        !SameParameterWithin(
            candidate.requested_target_parameter,
            candidate.effective_target_parameter,
            kTransitionMontageProjectedNoteTolerance);
    candidate.jog_walk_pair =
        IsJogWalkPair(source_parameter, requested_target_parameter);

    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        requested_target_parameter, artifact.metadata.frames_per_second);
    candidate.transition = EvaluateConfiguredTransition(
        artifact.skeleton, source_clip, target_clip, config);
    candidate.transition_class =
        candidate.accepted
            ? PoseSeamClass(candidate.transition.distance, config)
            : "REJECTED";

    for (const ReachableRegionAuditRow& row : reachable_data.rows) {
        if (SameParameterWithin(
                row.source_parameter, source_parameter,
                kTransitionMontageAnchorTolerance)) {
            candidate.source_coverage_available = true;
            candidate.source_coverage = row.support_coverage;
            break;
        }
    }

    std::ostringstream notes;
    notes << (candidate.accepted ? "accepted" : "rejected");
    if (candidate.source_is_anchor && source_support != nullptr) {
        notes << "; source " << NearestAnchorLabel(*source_support, source_parameter);
    }
    if (candidate.requested_target_is_anchor && target_support != nullptr) {
        notes << "; target "
              << NearestAnchorLabel(*target_support, requested_target_parameter);
    }
    if (candidate.jog_walk_pair) {
        notes << "; jog/walk pair";
    }
    if (candidate.requested_target_projected) {
        notes << "; requested target outside support";
    }
    if (candidate.effective_target_projected) {
        notes << "; runtime target projects to "
              << ParameterMd(candidate.effective_target_parameter);
    }
    if (candidate.source_coverage_available) {
        notes << "; source coverage " << candidate.source_coverage;
    }
    candidate.notes = notes.str();
    return candidate;
}

std::vector<TransitionAuditCandidate> BuildTransitionAuditCandidates(
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgEdge& edge,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const ReachableRegionAuditData& reachable_data,
    const pmg::ParameterSupport* source_support,
    const pmg::ParameterSupport* target_support,
    int samples_per_axis) {
    std::vector<pmg::ParameterVector> source_samples =
        BuildAuditParameterSamples(source_node.motion_space, samples_per_axis);
    std::vector<pmg::ParameterVector> target_samples =
        BuildAuditParameterSamples(target_node.motion_space, samples_per_axis);
    if (target_support != nullptr && target_support->Dimension() == 2) {
        AppendUniqueParameter(target_samples, {0.8f, 0.8f});
    }

    std::vector<TransitionAuditCandidate> candidates;
    for (const pmg::ParameterVector& source_parameter : source_samples) {
        for (const pmg::ParameterVector& target_parameter : target_samples) {
            candidates.push_back(BuildTransitionAuditCandidate(
                artifact, edge, source_node, target_node, config,
                reachable_data, source_parameter, target_parameter,
                source_support, target_support));
        }
    }
    return candidates;
}

TransitionMontageRow ToTransitionMontageRow(
    const TransitionAuditCandidate& candidate,
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const std::string& category,
    int category_rank,
    int manifest_index) {
    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        candidate.source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        candidate.requested_target_parameter, artifact.metadata.frames_per_second);
    const pmg::Pose source_pose = source_clip.frames[static_cast<std::size_t>(
        candidate.transition.source_frame)];
    const pmg::Pose aligned_target_pose = candidate.transition.alignment.Apply(
        target_clip.frames[static_cast<std::size_t>(
            candidate.transition.target_frame)]);

    TransitionMontageRow row;
    row.category = category;
    row.category_rank = category_rank;
    {
        std::ostringstream id;
        id << 'M' << std::setw(3) << std::setfill('0') << manifest_index;
        row.montage_id = id.str();
    }
    row.replay_id = row.montage_id;
    row.source_parameter = candidate.source_parameter;
    row.requested_target_parameter = candidate.requested_target_parameter;
    row.effective_target_parameter = candidate.effective_target_parameter;
    row.source_phase = candidate.transition.source_phase;
    row.target_phase = candidate.transition.target_phase;
    row.source_frame = candidate.transition.source_frame;
    row.target_frame = candidate.transition.target_frame;
    row.transition_distance = candidate.transition.distance;
    row.transition_class = candidate.transition_class;
    row.root_jump = RootJumpAtTransition(source_pose, aligned_target_pose);
    row.heading_jump =
        HeadingJumpAtTransition(source_pose, aligned_target_pose);
    row.velocity_jump = VelocityJumpAtTransition(
        source_clip, target_clip, candidate.transition.alignment,
        candidate.transition.source_frame, candidate.transition.target_frame);
    row.blend_frames = config.distance_grid.window_size;
    row.notes = candidate.notes;
    return row;
}

template <typename Predicate, typename RankKey>
void AppendRankedTransitionRows(
    const std::vector<TransitionAuditCandidate>& candidates,
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const std::string& category,
    int max_rows,
    Predicate predicate,
    RankKey rank_key,
    int& next_manifest_index,
    std::vector<TransitionMontageRow>& out_rows) {
    std::vector<const TransitionAuditCandidate*> ranked;
    for (const TransitionAuditCandidate& candidate : candidates) {
        if (predicate(candidate)) {
            ranked.push_back(&candidate);
        }
    }
    std::sort(
        ranked.begin(), ranked.end(),
        [&](const TransitionAuditCandidate* left,
            const TransitionAuditCandidate* right) {
            const auto left_key = rank_key(*left);
            const auto right_key = rank_key(*right);
            if (left_key != right_key) {
                return left_key > right_key;
            }
            if (left->transition.distance != right->transition.distance) {
                return left->transition.distance > right->transition.distance;
            }
            return ParameterMd(left->source_parameter) <
                   ParameterMd(right->source_parameter);
        });
    const int count =
        std::min(max_rows, static_cast<int>(ranked.size()));
    for (int index = 0; index < count; ++index) {
        out_rows.push_back(ToTransitionMontageRow(
            *ranked[static_cast<std::size_t>(index)], artifact,
            source_node, target_node, config, category, index + 1,
            next_manifest_index++));
    }
}

std::string TransitionMontageConclusion(
    const std::vector<TransitionAuditCandidate>& candidates,
    float bad_threshold) {
    int accepted_bad = 0;
    int accepted_near_threshold = 0;
    int rejected_jog_walk = 0;
    for (const TransitionAuditCandidate& candidate : candidates) {
        if (!candidate.accepted) {
            if (candidate.jog_walk_pair) {
                ++rejected_jog_walk;
            }
            continue;
        }
        if (candidate.transition.distance >= bad_threshold) {
            ++accepted_bad;
        } else if (candidate.transition.distance >= 0.9f * bad_threshold) {
            ++accepted_near_threshold;
        }
    }
    if (accepted_bad > 0) {
        return "FAIL_VISIBLE_TRANSITION_POP";
    }
    if (rejected_jog_walk >= 8) {
        return "FAIL_THRESHOLD_REJECTS_TOO_MUCH";
    }
    if (rejected_jog_walk > 0) {
        return "WARN_JOG_TRANSITION_WEAK";
    }
    if (accepted_near_threshold > 0) {
        return "WARN_MINOR_POP";
    }
    return "PASS_TRANSITIONS_VISUALLY_ACCEPTABLE";
}

TransitionMontageAuditData BuildTransitionMontageAudit(
    const pmg::BuiltPmgArtifact& artifact,
    const TransitionMontageAuditOptions& options) {
    TransitionMontageAuditData data;
    ReachableRegionAuditOptions reachable_options;
    reachable_options.pmg_path = options.pmg_path;
    reachable_options.output_csv = options.output_dir + "/_unused.csv";
    reachable_options.output_md = options.output_dir + "/_unused.md";
    reachable_options.output_dir = options.output_dir + "/_unused_maps";
    reachable_options.source_node = options.source_node;
    reachable_options.target_node = options.target_node;
    reachable_options.samples_per_axis = options.samples_per_axis;

    int edge_index = ResolveEdgeIndex(
        artifact, reachable_options, data.source_node, data.target_node);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* edge_metadata =
        FindEdgeBuildMetadata(artifact, data.source_node, data.target_node);
    if (edge_metadata == nullptr) {
        throw std::runtime_error(
            "--audit-transition-montage: missing edge build metadata");
    }
    data.config = edge_metadata->config;
    const ReachableRegionAuditData reachable_data =
        BuildReachableRegionAudit(artifact, reachable_options);

    std::filesystem::create_directories(options.output_dir);
    std::filesystem::remove_all(reachable_options.output_dir);
    const pmg::ParameterSupport* source_support =
        source_node.motion_space.HasExplicitParameterSupport()
            ? &*source_node.motion_space.ExplicitSupport()
            : nullptr;
    const pmg::ParameterSupport* target_support =
        target_node.motion_space.HasExplicitParameterSupport()
            ? &*target_node.motion_space.ExplicitSupport()
            : nullptr;

    const std::vector<TransitionAuditCandidate> candidates =
        BuildTransitionAuditCandidates(
            artifact, edge, source_node, target_node, data.config,
            reachable_data, source_support, target_support,
            options.samples_per_axis);

    data.unique_transition_count = static_cast<int>(candidates.size());
    data.accepted_count = static_cast<int>(std::count_if(
        candidates.begin(), candidates.end(), [](const auto& candidate) {
            return candidate.accepted;
        }));
    data.rejected_count = data.unique_transition_count - data.accepted_count;
    data.rejected_jog_walk_count = static_cast<int>(std::count_if(
        candidates.begin(), candidates.end(), [](const auto& candidate) {
            return !candidate.accepted && candidate.jog_walk_pair;
        }));
    data.conclusion = TransitionMontageConclusion(
        candidates, data.config.bad_transition_threshold);
    data.manifest_csv_path =
        options.output_dir + "/transition_montage_manifest.csv";

    int next_manifest_index = 1;
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "worst_accepted_default", kWorstAcceptedSamples,
        [](const auto& candidate) { return candidate.accepted; },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "near_threshold_accepted", kNearThresholdSamples,
        [](const auto& candidate) { return candidate.accepted; },
        [&](const auto& candidate) {
            return -std::abs(
                candidate.transition.distance -
                data.config.bad_transition_threshold);
        },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "rejected_high_d_jog_walk", kRejectedJogWalkSamples,
        [](const auto& candidate) {
            return !candidate.accepted && candidate.jog_walk_pair;
        },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "anchor_to_anchor", kAnchorToAnchorSamples,
        [](const auto& candidate) {
            return candidate.source_is_anchor &&
                   candidate.requested_target_is_anchor;
        },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "outside_request_projected", kOutsideRequestSamples,
        [](const auto& candidate) {
            return candidate.requested_target_projected;
        },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "source_dependent_shrinkage", kShrinkageSamples,
        [](const auto& candidate) {
            return candidate.source_coverage_available &&
                   (candidate.source_coverage <= 0.35f ||
                    SameParameterWithin(
                        candidate.source_parameter, {0.0f, 1.0f},
                        kTransitionMontageAnchorTolerance));
        },
        [](const auto& candidate) {
            return !candidate.accepted
                       ? candidate.transition.distance + 1000.0f
                       : candidate.transition.distance;
        },
        next_manifest_index, data.rows);
    return data;
}

void WriteTransitionMontageManifestCsv(
    const TransitionMontageAuditData& data) {
    std::ofstream csv(data.manifest_csv_path);
    if (!csv) {
        throw std::runtime_error(
            "--audit-transition-montage: cannot write manifest CSV");
    }
    csv << std::setprecision(9)
        << "category,category_rank,montage_id,replay_id,source_parameter,"
           "requested_target_parameter,effective_target_parameter,"
           "source_phase,source_frame,target_phase,target_frame,"
           "transition_metric_d,transition_class,root_jump,heading_jump,"
           "velocity_jump,blend_frames,notes\n";
    for (const TransitionMontageRow& row : data.rows) {
        csv << row.category << ',' << row.category_rank << ','
            << row.montage_id << ',' << row.replay_id << ','
            << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.requested_target_parameter) << ','
            << ParameterCsv(row.effective_target_parameter) << ','
            << row.source_phase << ',' << row.source_frame << ','
            << row.target_phase << ',' << row.target_frame << ','
            << row.transition_distance << ',' << row.transition_class << ','
            << row.root_jump << ',' << row.heading_jump << ','
            << row.velocity_jump << ',' << row.blend_frames << ",\""
            << row.notes << "\"\n";
    }
}

void WriteTransitionMontageMarkdown(
    const TransitionMontageAuditOptions& options,
    const TransitionMontageAuditData& data) {
    std::ofstream md(options.output_md);
    if (!md) {
        throw std::runtime_error(
            "--audit-transition-montage: cannot write markdown");
    }
    md << "# Transition Montage Audit\n\n";
    md << "## Purpose\n\n";
    md << "Report-only transition replay manifest for current PMG edge. "
          "No PMG artifact, threshold, or runtime behavior change. "
          "Conclusion uses current metric D plus root/heading/velocity seam "
          "diagnostics as proxy for later manual replay review.\n\n";
    md << "## Inputs\n\n";
    md << "- Artifact: `" << options.pmg_path << "`\n";
    md << "- Edge: `" << data.source_node << " -> " << data.target_node
       << "`\n";
    md << "- TGOOD/TBAD: `" << data.config.good_transition_threshold
       << " / " << data.config.bad_transition_threshold << "`\n";
    md << "- Blend frames: `" << data.config.distance_grid.window_size
       << "`\n";
    md << "- Unique source/target evaluations: " << data.unique_transition_count
       << "\n";
    md << "- Accepted / rejected requests: " << data.accepted_count << " / "
       << data.rejected_count << "\n";
    md << "- Rejected jog/walk requests: " << data.rejected_jog_walk_count
       << "\n";
    md << "- Conclusion: `" << data.conclusion << "`\n\n";
    md << "## Contract\n\n";
    md << "- `transition_metric_d` uses current offline build metric units.\n";
    md << "- `source_phase` / `target_phase` come from measured transition search.\n";
    md << "- `requested_target_parameter` is audited request.\n";
    md << "- `effective_target_parameter` is current runtime clamp/project result.\n";
    md << "- `REJECTED` means request fell outside interpolated target box for current source sample.\n";
    md << "- Manifest ids are deterministic for identical artifact + sampling.\n\n";

    const std::array<std::string, 6> categories = {
        "worst_accepted_default",
        "near_threshold_accepted",
        "rejected_high_d_jog_walk",
        "anchor_to_anchor",
        "outside_request_projected",
        "source_dependent_shrinkage"};
    for (const std::string& category : categories) {
        md << "## " << category << "\n\n";
        md << "| Rank | Montage | Source p | Req target p | Eff target p | Src phase/frame | Tgt phase/frame | D | Class | Root jump | Heading jump | Velocity jump | Notes |\n";
        md << "|---:|---|---|---|---|---|---|---:|---|---:|---:|---:|---|\n";
        for (const TransitionMontageRow& row : data.rows) {
            if (row.category != category) {
                continue;
            }
            md << "| " << row.category_rank << " | `" << row.montage_id
               << "` | " << ParameterMd(row.source_parameter) << " | "
               << ParameterMd(row.requested_target_parameter) << " | "
               << ParameterMd(row.effective_target_parameter) << " | "
               << row.source_phase << " / " << row.source_frame << " | "
               << row.target_phase << " / " << row.target_frame << " | "
               << row.transition_distance << " | `" << row.transition_class
               << "` | " << row.root_jump << " | " << row.heading_jump
               << " | " << row.velocity_jump << " | " << row.notes
               << " |\n";
        }
        md << "\n";
    }
    md << "## Artifacts\n\n";
    md << "- Markdown report: `" << options.output_md << "`\n";
    md << "- Manifest CSV: `" << data.manifest_csv_path << "`\n";
    md << "- Montage directory: `" << options.output_dir << "`\n";
}

struct ContactTransitionRow {
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector target_parameter;
    float transition_distance = 0.0f;
    int contact_mismatch_count = 0;
    float max_contact_foot_velocity = 0.0f;
    float total_skate_distance = 0.0f;
    float left_contact_confidence = 0.0f;
    float right_contact_confidence = 0.0f;
};

bool FrameInContact(
    const std::vector<pmg::ContactInterval>& contacts, int joint, int frame) {
    return std::any_of(contacts.begin(), contacts.end(), [&](const auto& contact) {
        return contact.joint_index == joint && frame >= contact.first_frame &&
               frame <= contact.last_frame;
    });
}

std::vector<ContactTransitionRow> BuildContactTransitionRows(
    const pmg::BuiltPmgArtifact& artifact,
    const ContactTransitionAuditOptions& options) {
    ReachableRegionAuditOptions reachable_options;
    reachable_options.source_node = options.source_node;
    reachable_options.target_node = options.target_node;
    std::string source_name;
    std::string target_name;
    const int edge_index = ResolveEdgeIndex(
        artifact, reachable_options, source_name, target_name);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* metadata =
        FindEdgeBuildMetadata(artifact, source_name, target_name);
    if (metadata == nullptr) {
        throw std::runtime_error(
            "--audit-contact-transitions: missing edge build metadata");
    }
    std::vector<int> contact_joints =
        metadata->config.transition_metric.contact_joint_indices;
    if (contact_joints.empty()) {
        const auto registration = std::find_if(
            artifact.metadata.node_registrations.begin(),
            artifact.metadata.node_registrations.end(),
            [&](const auto& value) { return value.node_name == source_name; });
        if (registration != artifact.metadata.node_registrations.end()) {
            for (const std::string& joint_name : registration->contact_joints) {
                const int joint = ResolveJointIndex(artifact.skeleton, joint_name);
                if (joint >= 0) {
                    contact_joints.push_back(joint);
                }
            }
        }
    }
    if (contact_joints.empty()) {
        for (const char* joint_name : {"LeftAnkle", "RightAnkle"}) {
            const int joint = ResolveJointIndex(artifact.skeleton, joint_name);
            if (joint >= 0) {
                contact_joints.push_back(joint);
            }
        }
    }
    if (contact_joints.size() < 2) {
        throw std::runtime_error(
            "--audit-contact-transitions: edge needs two contact joints");
    }

    std::vector<ContactTransitionRow> rows;
    const auto source_samples = BuildAuditParameterSamples(
        source_node.motion_space, options.samples_per_axis);
    const auto target_samples = BuildAuditParameterSamples(
        target_node.motion_space, options.samples_per_axis);
    for (const auto& source_parameter : source_samples) {
        for (const auto& target_parameter : target_samples) {
            const auto lookup = edge.LookupInterpolated(
                source_parameter, target_parameter);
            if (!lookup.has_value() ||
                !lookup->target_parameter_box.Contains(target_parameter)) {
                continue;
            }
            const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
                source_parameter, artifact.metadata.frames_per_second);
            const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
                target_parameter, artifact.metadata.frames_per_second);
            const pmg::OptimalTransition transition = EvaluateConfiguredTransition(
                artifact.skeleton, source_clip, target_clip, metadata->config);
            const pmg::ContactDetectionSettings source_settings =
                pmg::EstimateContactSettings(
                    artifact.skeleton, source_clip, contact_joints);
            const pmg::ContactDetectionSettings target_settings =
                pmg::EstimateContactSettings(
                    artifact.skeleton, target_clip, contact_joints);
            const auto source_contacts = pmg::DetectContacts(
                artifact.skeleton, source_clip, contact_joints, source_settings);
            const auto target_contacts = pmg::DetectContacts(
                artifact.skeleton, target_clip, contact_joints, target_settings);

            ContactTransitionRow row;
            row.source_parameter = source_parameter;
            row.target_parameter = target_parameter;
            row.transition_distance = transition.distance;
            std::array<int, 2> contact_frames{};
            std::array<int, 2> blend_frames{};
            std::array<pmg::Vec3, 2> previous_positions{};
            const int frame_count = metadata->config.distance_grid.window_size;
            for (int blend_frame = 0; blend_frame < frame_count; ++blend_frame) {
                const int source_frame = std::clamp(
                    transition.source_frame + blend_frame, 0,
                    source_clip.NumFrames() - 1);
                const int target_frame = std::clamp(
                    transition.target_frame - frame_count + 1 + blend_frame, 0,
                    target_clip.NumFrames() - 1);
                const float linear_alpha = frame_count == 1
                    ? 1.0f
                    : static_cast<float>(blend_frame) /
                          static_cast<float>(frame_count - 1);
                const float alpha = linear_alpha * linear_alpha *
                                    (3.0f - 2.0f * linear_alpha);
                const pmg::Pose blended = pmg::BlendPose(
                    source_clip.frames[source_frame],
                    transition.alignment.Apply(target_clip.frames[target_frame]),
                    alpha);
                const auto positions = pmg::ComputeJointWorldPositions(
                    artifact.skeleton, blended);
                for (int foot = 0; foot < 2; ++foot) {
                    const int joint = contact_joints[foot];
                    const bool source_contact = FrameInContact(
                        source_contacts, joint, source_frame);
                    const bool target_contact = FrameInContact(
                        target_contacts, joint, target_frame);
                    if (source_contact != target_contact) {
                        ++row.contact_mismatch_count;
                    }
                    const bool in_contact = source_contact || target_contact;
                    ++blend_frames[foot];
                    if (in_contact) {
                        ++contact_frames[foot];
                    }
                    if (blend_frame > 0 && in_contact) {
                        const float step = HorizontalLength(
                            positions[joint] - previous_positions[foot]);
                        row.total_skate_distance += step;
                        row.max_contact_foot_velocity = std::max(
                            row.max_contact_foot_velocity,
                            step * artifact.metadata.frames_per_second);
                    }
                    previous_positions[foot] = positions[joint];
                }
            }
            row.left_contact_confidence = static_cast<float>(contact_frames[0]) /
                                          static_cast<float>(blend_frames[0]);
            row.right_contact_confidence = static_cast<float>(contact_frames[1]) /
                                           static_cast<float>(blend_frames[1]);
            rows.push_back(row);
        }
    }
    return rows;
}

std::string ContactTransitionConclusion(
    const std::vector<ContactTransitionRow>& rows) {
    const int mismatched = static_cast<int>(std::count_if(
        rows.begin(), rows.end(), [](const auto& row) {
            return row.contact_mismatch_count > 0;
        }));
    if (mismatched > static_cast<int>(rows.size() / 2)) {
        return "WARN_CONTACT_MISMATCH_COMMON";
    }
    if (mismatched > 0) {
        return "WARN_FOOT_SLIDING_VISIBLE";
    }
    return "PASS_CONTACT_ARTIFACT_LOW";
}

void WriteContactTransitionAudit(
    const ContactTransitionAuditOptions& options,
    const std::vector<ContactTransitionRow>& rows) {
    std::ofstream csv(options.output_csv);
    std::ofstream md(options.output_md);
    if (!csv || !md) {
        throw std::runtime_error(
            "--audit-contact-transitions: cannot write output");
    }
    csv << std::setprecision(9)
        << "source_parameter,target_parameter,transition_metric_d,"
           "contact_mismatch_count,max_contact_foot_velocity,"
           "total_skate_distance,left_contact_confidence,"
           "right_contact_confidence\n";
    for (const auto& row : rows) {
        csv << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.target_parameter) << ','
            << row.transition_distance << ',' << row.contact_mismatch_count << ','
            << row.max_contact_foot_velocity << ',' << row.total_skate_distance
            << ',' << row.left_contact_confidence << ','
            << row.right_contact_confidence << '\n';
    }
    std::vector<ContactTransitionRow> ranked = rows;
    std::sort(ranked.begin(), ranked.end(), [](const auto& left, const auto& right) {
        return left.total_skate_distance > right.total_skate_distance;
    });
    md << "# Contact Transition Audit\n\n"
       << "Report-only contact/foot-slide diagnostics. Distances use motion "
          "corpus units; velocity uses corpus units/s; confidence is contact "
          "frame fraction over blend. Contact is estimated independently per "
          "generated clip. No runtime or transition rejection change.\n\n"
       << "- Evaluated accepted transitions: " << rows.size() << "\n"
       << "- Conclusion: `" << ContactTransitionConclusion(rows) << "`\n\n"
       << "| Rank | Source | Target | D | Mismatch frames | Max foot velocity | Skate distance | Left confidence | Right confidence |\n"
       << "|---:|---|---|---:|---:|---:|---:|---:|---:|\n";
    const int count = std::min(20, static_cast<int>(ranked.size()));
    for (int index = 0; index < count; ++index) {
        const auto& row = ranked[index];
        md << "| " << index + 1 << " | " << ParameterMd(row.source_parameter)
           << " | " << ParameterMd(row.target_parameter) << " | "
           << row.transition_distance << " | " << row.contact_mismatch_count
           << " | " << row.max_contact_foot_velocity << " | "
           << row.total_skate_distance << " | " << row.left_contact_confidence
           << " | " << row.right_contact_confidence << " |\n";
    }
}

constexpr float kAcceptanceNearThresholdRatio = 0.9f;
constexpr float kShrinkageCoverageWarnThreshold = 0.35f;
constexpr float kKnownBadCaseTolerance = 1.0e-5f;

bool IsKnownAcceptedBadCase(
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& target_parameter) {
    const bool original_b1 = SameParameterWithin(
        source_parameter, {0.1875f, 0.375f}, kKnownBadCaseTolerance) &&
        SameParameterWithin(
            target_parameter, {0.025f, 0.875f}, kKnownBadCaseTolerance);
    const bool fail_vis_001 = SameParameterWithin(
        source_parameter, {-0.04f, 0.8f}, kKnownBadCaseTolerance) &&
        SameParameterWithin(
            target_parameter, {0.5f, 0.0f}, kKnownBadCaseTolerance);
    return original_b1 || fail_vis_001;
}

std::string BuildAcceptanceConsistencyNotes(
    const TransitionAuditCandidate& candidate,
    float bad_threshold) {
    std::vector<std::string> notes;
    if (candidate.accepted &&
        candidate.transition.distance >= bad_threshold) {
        notes.push_back(
            "accepted by interpolated target box but measured D is BAD");
        if (!candidate.effective_target_projected) {
            notes.push_back(
                "likely box overreach: acceptance gates box membership, not D");
        }
    } else if (candidate.accepted &&
               candidate.transition.distance >=
                   kAcceptanceNearThresholdRatio * bad_threshold) {
        notes.push_back("accepted near TBAD; small box change may flip class");
    }
    if (candidate.source_coverage_available &&
        candidate.source_coverage <= kShrinkageCoverageWarnThreshold) {
        notes.push_back("source-dependent shrinkage from reachable-region audit");
    }
    if (candidate.jog_walk_pair) {
        notes.push_back("jog/walk speed-mismatch case");
    }
    if (candidate.effective_target_projected) {
        notes.push_back("runtime projects request inside current box/support");
    }
    if (IsKnownAcceptedBadCase(
            candidate.source_parameter,
            candidate.requested_target_parameter)) {
        notes.push_back("known B1 failing case");
    }
    if (notes.empty()) {
        notes.push_back("box and measured D agree");
    }

    std::ostringstream out;
    for (std::size_t index = 0; index < notes.size(); ++index) {
        if (index > 0) {
            out << "; ";
        }
        out << notes[index];
    }
    return out.str();
}

// Strict-interior box membership for the acceptance-consistency audit only.
// Runtime ParameterAabb::Contains is inclusive (a point on a face is accepted);
// a BAD target that is accepted ONLY because it lies exactly on a box face is a
// measure-zero boundary artifact, not interior edge-box overreach. The audit is
// therefore made stricter than the runtime lookup: it treats face-touching BAD
// targets as not-accepted so they do not register as overreach. No epsilon
// margin is applied, so genuinely interior BAD targets (any depth past the face)
// still count -- the acceptance tolerance is not relaxed.
bool ContainsStrictInterior(
    const pmg::ParameterAabb& box, const pmg::ParameterVector& parameter) {
    if (box.IsEmpty() ||
        box.min_corner.size() != parameter.size() ||
        box.max_corner.size() != parameter.size()) {
        return false;
    }
    for (std::size_t index = 0; index < parameter.size(); ++index) {
        if (parameter[index] <= box.min_corner[index] ||
            parameter[index] >= box.max_corner[index]) {
            return false;
        }
    }
    return true;
}

TransitionAcceptanceConsistencyRow BuildAcceptanceConsistencyRow(
    const TransitionAuditCandidate& candidate,
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const pmg::TransitionQualityGateConfig& quality_gate,
    const pmg::ParameterSupport* source_support,
    const pmg::ParameterSupport* target_support) {
    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        candidate.source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        candidate.requested_target_parameter, artifact.metadata.frames_per_second);
    const pmg::Pose source_pose = source_clip.frames[static_cast<std::size_t>(
        candidate.transition.source_frame)];
    const pmg::Pose aligned_target_pose = candidate.transition.alignment.Apply(
        target_clip.frames[static_cast<std::size_t>(
            candidate.transition.target_frame)]);

    TransitionAcceptanceConsistencyRow row;
    row.source_parameter = candidate.source_parameter;
    row.requested_target_parameter = candidate.requested_target_parameter;
    row.effective_target_parameter = candidate.effective_target_parameter;
    row.interpolated_target_box = candidate.interpolated_target_box;
    row.accepted_by_box = candidate.accepted;
    row.transition_distance = candidate.transition.distance;
    row.metric_class = candidate.transition_class;
    // Exclusive-boundary audit: a BAD target accepted only because it sits on a
    // box face is a boundary artifact of the inclusive runtime lookup, not edge-
    // box overreach. Count overreach only for strict-interior BAD targets.
    row.acceptance_violation =
        ContainsStrictInterior(
            candidate.interpolated_target_box,
            candidate.requested_target_parameter) &&
        candidate.transition.distance >= config.bad_transition_threshold;
    row.distance_over_tbad =
        candidate.transition.distance - config.bad_transition_threshold;
    row.source_coverage_available = candidate.source_coverage_available;
    row.source_coverage = candidate.source_coverage;
    row.source_phase = candidate.transition.source_phase;
    row.source_frame = candidate.transition.source_frame;
    row.target_phase = candidate.transition.target_phase;
    row.target_frame = candidate.transition.target_frame;
    row.root_jump = RootJumpAtTransition(source_pose, aligned_target_pose);
    row.heading_jump =
        HeadingJumpAtTransition(source_pose, aligned_target_pose);
    row.velocity_jump = VelocityJumpAtTransition(
        source_clip, target_clip, candidate.transition.alignment,
        candidate.transition.source_frame, candidate.transition.target_frame);
    if (candidate.accepted) {
        row.quality = pmg::MeasureExactTransitionQuality(
            artifact, source_node, target_node, config,
            candidate.source_parameter, candidate.effective_target_parameter,
            candidate.transition, 3, 3);
        const pmg::TransitionQualityGateDecision decision =
            pmg::EvaluateTransitionQualityGate(*row.quality, quality_gate);
        const bool no_runtime_transition = &source_node == &target_node &&
            candidate.source_parameter == candidate.effective_target_parameter;
        row.quality_gate_accepts = no_runtime_transition || decision.accepted;
        row.quality_reject_reason = no_runtime_transition
            ? "none"
            : pmg::TransitionQualityGateReasonName(decision.reason);
    }
    row.nearest_source_anchor_label =
        source_support != nullptr
            ? NearestAnchorLabel(*source_support, candidate.source_parameter)
            : "";
    row.nearest_target_anchor_label =
        target_support != nullptr
            ? NearestAnchorLabel(
                  *target_support, candidate.requested_target_parameter)
            : "";
    row.jog_walk_pair = candidate.jog_walk_pair;
    row.notes = BuildAcceptanceConsistencyNotes(
        candidate, config.bad_transition_threshold);
    return row;
}

std::string NearestMetricEvidence(
    const TransitionAcceptanceConsistencyRow& row,
    const std::vector<TransitionAuditCandidate>& candidates,
    float good_threshold,
    float bad_threshold,
    bool find_good) {
    const TransitionAuditCandidate* nearest = nullptr;
    float nearest_distance = std::numeric_limits<float>::infinity();
    for (const TransitionAuditCandidate& candidate : candidates) {
        if (!SameParameterWithin(
                candidate.source_parameter, row.source_parameter,
                kTransitionMontageAnchorTolerance)) {
            continue;
        }
        const bool matches = find_good
                                 ? candidate.transition.distance <= good_threshold
                                 : candidate.transition.distance >= bad_threshold;
        const float parameter_distance = pmg::SquaredDistance(
            candidate.requested_target_parameter,
            row.requested_target_parameter);
        if (matches && parameter_distance < nearest_distance) {
            nearest = &candidate;
            nearest_distance = parameter_distance;
        }
    }
    if (nearest == nullptr) {
        return "none";
    }
    std::ostringstream evidence;
    evidence << "p=" << ParameterMd(nearest->requested_target_parameter)
             << ", D=" << nearest->transition.distance;
    return evidence.str();
}

std::string OverreachOrigin(
    const TransitionAcceptanceConsistencyRow& row,
    const pmg::PmgEdge& edge) {
    if (!row.acceptance_violation) {
        return "none";
    }
    float nearest_source_distance = std::numeric_limits<float>::infinity();
    for (const pmg::TransitionSample& sample : edge.samples) {
        nearest_source_distance = std::min(
            nearest_source_distance,
            pmg::SquaredDistance(
                sample.source_parameter, row.source_parameter));
    }
    for (const pmg::TransitionSample& sample : edge.samples) {
        const float source_distance = pmg::SquaredDistance(
            sample.source_parameter, row.source_parameter);
        if (std::abs(source_distance - nearest_source_distance) <=
                kTransitionMontageAnchorTolerance &&
            sample.target_parameter_box.Contains(
                row.requested_target_parameter)) {
            return "box_construction";
        }
    }
    return "interpolation";
}

template <typename Predicate, typename RankKey>
void WriteAcceptanceConsistencySection(
    std::ofstream& md,
    const std::string& title,
    const std::vector<TransitionAcceptanceConsistencyRow>& rows,
    Predicate predicate,
    RankKey rank_key) {
    std::vector<const TransitionAcceptanceConsistencyRow*> ranked;
    for (const TransitionAcceptanceConsistencyRow& row : rows) {
        if (predicate(row)) {
            ranked.push_back(&row);
        }
    }
    std::sort(
        ranked.begin(), ranked.end(),
        [&](const TransitionAcceptanceConsistencyRow* left,
            const TransitionAcceptanceConsistencyRow* right) {
            const auto left_key = rank_key(*left);
            const auto right_key = rank_key(*right);
            if (left_key != right_key) {
                return left_key > right_key;
            }
            if (left->transition_distance != right->transition_distance) {
                return left->transition_distance > right->transition_distance;
            }
            return ParameterMd(left->source_parameter) <
                   ParameterMd(right->source_parameter);
        });

    md << "## " << title << "\n\n";
    if (ranked.empty()) {
        md << "No rows.\n\n";
        return;
    }
    md << "| Rank | Source p | Req target p | Target box | Box | Quality | Reject reason | D | Class | D-TBAD | Nearest GOOD | Nearest BAD | Origin | Source coverage | Notes |\n";
    md << "|---:|---|---|---|---|---|---|---:|---|---:|---|---|---|---:|---|\n";
    for (std::size_t index = 0; index < ranked.size(); ++index) {
        const TransitionAcceptanceConsistencyRow& row =
            *ranked[index];
        md << "| " << (index + 1) << " | "
           << ParameterMd(row.source_parameter) << " | "
           << ParameterMd(row.requested_target_parameter) << " | "
           << ParameterMd(row.interpolated_target_box.min_corner) << " .. "
           << ParameterMd(row.interpolated_target_box.max_corner) << " | "
           << (row.accepted_by_box ? "`true`" : "`false`") << " | "
           << (row.quality_gate_accepts ? "`accept`" : "`reject`") << " | `"
           << row.quality_reject_reason << "` | "
           << row.transition_distance << " | `" << row.metric_class
           << "` | " << row.distance_over_tbad << " | "
           << row.nearest_good_evidence << " | "
           << row.nearest_bad_evidence << " | `"
           << row.overreach_origin << "` | ";
        if (row.source_coverage_available) {
            md << row.source_coverage;
        } else {
            md << "n/a";
        }
        md << " | " << row.notes << " |\n";
    }
    md << "\n";
}

TransitionAcceptanceConsistencyAuditData BuildTransitionAcceptanceConsistencyAudit(
    const pmg::BuiltPmgArtifact& artifact,
    const TransitionAcceptanceConsistencyAuditOptions& options) {
    TransitionAcceptanceConsistencyAuditData data;
    ReachableRegionAuditOptions reachable_options;
    reachable_options.pmg_path = options.pmg_path;
    const std::filesystem::path output_csv_path(options.output_csv);
    const std::filesystem::path temp_root =
        output_csv_path.has_parent_path()
            ? output_csv_path.parent_path()
            : std::filesystem::path(".");
    reachable_options.output_csv =
        (temp_root / "_acceptance_consistency_reachable_unused.csv").string();
    reachable_options.output_md =
        (temp_root / "_acceptance_consistency_reachable_unused.md").string();
    reachable_options.output_dir =
        (temp_root / "_acceptance_consistency_reachable_unused_maps").string();
    reachable_options.source_node = options.source_node;
    reachable_options.target_node = options.target_node;
    reachable_options.samples_per_axis = options.samples_per_axis;

    int edge_index = ResolveEdgeIndex(
        artifact, reachable_options, data.source_node, data.target_node);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* edge_metadata =
        FindEdgeBuildMetadata(artifact, data.source_node, data.target_node);
    if (edge_metadata == nullptr) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: missing edge build metadata");
    }
    data.config = edge_metadata->config;
    const ReachableRegionAuditData reachable_data =
        BuildReachableRegionAudit(artifact, reachable_options);
    std::filesystem::remove(reachable_options.output_csv);
    std::filesystem::remove(reachable_options.output_md);
    std::filesystem::remove_all(reachable_options.output_dir);
    const pmg::ParameterSupport* source_support =
        source_node.motion_space.HasExplicitParameterSupport()
            ? &*source_node.motion_space.ExplicitSupport()
            : nullptr;
    const pmg::ParameterSupport* target_support =
        target_node.motion_space.HasExplicitParameterSupport()
            ? &*target_node.motion_space.ExplicitSupport()
            : nullptr;
    const std::vector<TransitionAuditCandidate> candidates =
        BuildTransitionAuditCandidates(
            artifact, edge, source_node, target_node, data.config,
            reachable_data, source_support, target_support,
            options.samples_per_axis);

    bool any_box_overreach = false;
    for (const TransitionAuditCandidate& candidate : candidates) {
        TransitionAcceptanceConsistencyRow row =
            BuildAcceptanceConsistencyRow(
                candidate, artifact, source_node, target_node, data.config,
                options.quality_gate,
                source_support, target_support);
        row.nearest_good_evidence = NearestMetricEvidence(
            row, candidates, data.config.good_transition_threshold,
            data.config.bad_transition_threshold, true);
        row.nearest_bad_evidence = NearestMetricEvidence(
            row, candidates, data.config.good_transition_threshold,
            data.config.bad_transition_threshold, false);
        row.overreach_origin = OverreachOrigin(row, edge);
        data.accepted_bad_count += row.acceptance_violation ? 1 : 0;
        data.accepted_bad_after_quality_gate +=
            row.acceptance_violation && row.quality_gate_accepts ? 1 : 0;
        data.box_accepted_quality_rejected +=
            row.accepted_by_box && !row.quality_gate_accepts ? 1 : 0;
        if (row.accepted_by_box && !row.quality_gate_accepts) {
            ++data.quality_reject_reasons[row.quality_reject_reason];
        }
        data.near_threshold_accepted_count +=
            row.accepted_by_box &&
                    row.transition_distance <
                        data.config.bad_transition_threshold &&
                    row.transition_distance >=
                        kAcceptanceNearThresholdRatio *
                            data.config.bad_transition_threshold
                ? 1
                : 0;
        data.accepted_near_threshold_after_quality_gate +=
            row.accepted_by_box && row.quality_gate_accepts &&
                    row.transition_distance < data.config.bad_transition_threshold &&
                    row.transition_distance >=
                        kAcceptanceNearThresholdRatio *
                            data.config.bad_transition_threshold
                ? 1
                : 0;
        data.jog_walk_row_count += row.jog_walk_pair ? 1 : 0;
        data.shrinkage_row_count +=
            row.source_coverage_available &&
                    row.source_coverage <=
                        kShrinkageCoverageWarnThreshold
                ? 1
                : 0;
        data.known_bad_case_found =
            data.known_bad_case_found ||
            IsKnownAcceptedBadCase(
                row.source_parameter, row.requested_target_parameter);
        any_box_overreach =
            any_box_overreach ||
            (row.acceptance_violation &&
             SameParameterWithin(
                 row.requested_target_parameter,
                 row.effective_target_parameter,
                 kTransitionMontageProjectedNoteTolerance));
        data.rows.push_back(std::move(row));
    }

    if (data.accepted_bad_count > 0) {
        data.consistency_conclusion = "FAIL_OVERREACH_REMAINS";
    } else if (data.shrinkage_row_count > 0) {
        data.consistency_conclusion = "PASS_BUT_COVERAGE_SHRUNK";
    } else {
        data.consistency_conclusion = "PASS_NO_ACCEPTED_BAD_TRANSITIONS";
    }

    if (any_box_overreach) {
        data.root_cause_conclusion = "FAIL_EDGE_BOX_OVERREACH";
    } else {
        data.root_cause_conclusion = data.consistency_conclusion;
    }
    return data;
}

void WriteTransitionAcceptanceConsistencyCsv(
    const TransitionAcceptanceConsistencyAuditOptions& options,
    const TransitionAcceptanceConsistencyAuditData& data) {
    std::ofstream csv(options.output_csv);
    if (!csv) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: cannot write CSV");
    }
    csv << std::setprecision(9)
        << "source_parameter,requested_target_parameter,effective_target_parameter,"
           "interpolated_target_box_min,interpolated_target_box_max,"
           "accepted_by_box,transition_metric_d,metric_class,acceptance_violation,"
           "distance_over_tbad,source_coverage,source_phase,source_frame,"
           "target_phase,target_frame,root_jump,heading_jump,velocity_jump,"
           "root_speed_ratio,yaw_rate_ratio,left_foot_drift,right_foot_drift,"
           "left_foot_height_before,left_foot_height_after,"
           "right_foot_height_before,right_foot_height_after,"
           "left_contact_before,left_contact_after,right_contact_before,"
           "right_contact_after,quality_gate_accepts,quality_reject_reason,"
           "nearest_source_anchor_label,nearest_target_anchor_label,"
           "nearest_good_evidence,nearest_bad_evidence,overreach_origin,notes\n";
    for (const TransitionAcceptanceConsistencyRow& row : data.rows) {
        csv << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.requested_target_parameter) << ','
            << ParameterCsv(row.effective_target_parameter) << ','
            << ParameterCsv(row.interpolated_target_box.min_corner) << ','
            << ParameterCsv(row.interpolated_target_box.max_corner) << ','
            << (row.accepted_by_box ? "true" : "false") << ','
            << row.transition_distance << ',' << row.metric_class << ','
            << (row.acceptance_violation ? "true" : "false") << ','
            << row.distance_over_tbad << ',';
        if (row.source_coverage_available) {
            csv << row.source_coverage;
        }
        csv << ',' << row.source_phase << ',' << row.source_frame << ','
            << row.target_phase << ',' << row.target_frame << ','
            << row.root_jump << ',' << row.heading_jump << ','
            << row.velocity_jump << ',';
        if (row.quality.has_value()) {
            const pmg::TransitionQualityRecord& quality = *row.quality;
            csv << quality.root_speed_ratio << ',' << quality.yaw_rate_ratio << ','
                << quality.left_foot_drift << ',' << quality.right_foot_drift << ','
                << quality.left_foot_height_before << ','
                << quality.left_foot_height_after << ','
                << quality.right_foot_height_before << ','
                << quality.right_foot_height_after << ','
                << pmg::TransitionContactStateName(quality.left_contact_before)
                << ',' << pmg::TransitionContactStateName(quality.left_contact_after)
                << ',' << pmg::TransitionContactStateName(quality.right_contact_before)
                << ',' << pmg::TransitionContactStateName(quality.right_contact_after);
        } else {
            csv << ",,,,,,,,,,,";
        }
        csv << ',' << (row.quality_gate_accepts ? "true" : "false") << ','
            << row.quality_reject_reason << ",\""
            << row.nearest_source_anchor_label << "\",\""
            << row.nearest_target_anchor_label << "\",\""
            << row.nearest_good_evidence << "\",\""
            << row.nearest_bad_evidence << "\","
            << row.overreach_origin << ",\""
            << row.notes << "\"\n";
    }
}

void WriteTransitionAcceptanceConsistencyMarkdown(
    const TransitionAcceptanceConsistencyAuditOptions& options,
    const TransitionAcceptanceConsistencyAuditData& data) {
    std::ofstream md(options.output_md);
    if (!md) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: cannot write markdown");
    }
    md << "# Transition Acceptance Consistency Audit\n\n";
    md << "## Purpose\n\n";
    md << "Gate interpolated edge acceptance against sampled transition "
          "quality under current PMG artifact. Any accepted row with "
          "D >= TBAD fails command.\n\n";
    md << "## Inputs\n\n";
    md << "- Artifact: `" << options.pmg_path << "`\n";
    md << "- Edge: `" << data.source_node << " -> " << data.target_node
       << "`\n";
    md << "- TGOOD/TBAD: `" << data.config.good_transition_threshold
       << " / " << data.config.bad_transition_threshold << "`\n";
    md << "- Evaluated source/target rows: " << data.rows.size() << "\n";
    md << "- Accepted BAD transitions: " << data.accepted_bad_count << "\n";
    md << "- Accepted BAD after quality gate: "
       << data.accepted_bad_after_quality_gate << "\n";
    md << "- Near-threshold accepted transitions: "
       << data.near_threshold_accepted_count << "\n";
    md << "- Near-threshold accepted after quality gate: "
       << data.accepted_near_threshold_after_quality_gate << "\n";
    md << "- Box-accepted, quality-rejected: "
       << data.box_accepted_quality_rejected << "\n";
    md << "- Quality reject reasons: ";
    if (data.quality_reject_reasons.empty()) {
        md << "`none`";
    } else {
        bool first = true;
        for (const auto& [reason, count] : data.quality_reject_reasons) {
            if (!first) md << ", ";
            md << '`' << reason << "=" << count << '`';
            first = false;
        }
    }
    md << "\n";
    md << "- Known bad case found: "
       << (data.known_bad_case_found ? "`yes`" : "`no`") << "\n";
    md << "- Consistency conclusion: `" << data.consistency_conclusion
       << "`\n";
    md << "- Root-cause conclusion: `" << data.root_cause_conclusion
       << "`\n\n";
    md << "## Root Cause\n\n";
    md << "- Acceptance means requested target lies inside interpolated "
          "target box.\n";
    md << "- Measured `D` is evaluated independently at requested target.\n";
    md << "- Each row reports interpolated box, nearest sampled GOOD/BAD "
          "evidence at same source parameter, and overreach origin.\n";
    md << "- `box_construction` means target is inside a stored sampled box; "
          "`interpolation` means only interpolated lookup introduced it.\n\n";

    WriteAcceptanceConsistencySection(
        md, "Accepted BAD transitions", data.rows,
        [](const auto& row) { return row.acceptance_violation; },
        [](const auto& row) { return row.distance_over_tbad; });
    WriteAcceptanceConsistencySection(
        md, "Near-threshold accepted transitions", data.rows,
        [&](const auto& row) {
            return row.accepted_by_box &&
                   row.transition_distance < data.config.bad_transition_threshold &&
                   row.distance_over_tbad >=
                       -(1.0f - kAcceptanceNearThresholdRatio) *
                           data.config.bad_transition_threshold;
        },
        [](const auto& row) { return -std::abs(row.distance_over_tbad); });
    WriteAcceptanceConsistencySection(
        md, "Known bad case", data.rows,
        [](const auto& row) {
            return IsKnownAcceptedBadCase(
                row.source_parameter, row.requested_target_parameter);
        },
        [](const auto& row) { return row.transition_distance; });
    WriteAcceptanceConsistencySection(
        md, "Source-dependent shrinkage focus", data.rows,
        [](const auto& row) {
            return (row.source_coverage_available &&
                    row.source_coverage <=
                        kShrinkageCoverageWarnThreshold) ||
                   SameParameterWithin(
                       row.source_parameter, {0.0f, 1.0f},
                       kKnownBadCaseTolerance);
        },
        [](const auto& row) { return row.transition_distance; });
    WriteAcceptanceConsistencySection(
        md, "Jog/walk focus", data.rows,
        [](const auto& row) { return row.jog_walk_pair; },
        [](const auto& row) { return row.transition_distance; });
    md << "## Artifacts\n\n";
    md << "- CSV: `" << options.output_csv << "`\n";
    md << "- Markdown: `" << options.output_md << "`\n";
}

using TransitionProbeResult = pmg::TransitionProbeResult;

bool ContactMismatch(
    pmg::TransitionContactState before,
    pmg::TransitionContactState after) {
    return before != pmg::TransitionContactState::kUnknown &&
           after != pmg::TransitionContactState::kUnknown && before != after;
}

TransitionProbeResult BuildTransitionProbe(
    const pmg::BuiltPmgArtifact& artifact,
    const TransitionProbeOptions& options) {
    pmg::TransitionProbeRequest request;
    request.source_node = options.source_node;
    request.target_node = options.target_node;
    request.source_parameter = options.source_parameter;
    request.requested_target_parameter = options.target_parameter;
    request.frames_before = options.frames_before;
    request.frames_after = options.frames_after;
    request.quality_gate = options.quality_gate;
    return pmg::ProbeTransition(artifact, request);
}

void WriteTransitionProbeCsv(
    const std::string& path,
    const TransitionProbeResult& row) {
    std::ofstream csv(path);
    if (!csv) throw std::runtime_error("--probe-transition: cannot write CSV");
    csv << std::setprecision(9)
        << "source_parameter,requested_target_parameter,effective_target_parameter,"
           "target_box_min,target_box_max,accepted_by_box,transition_metric_d,"
           "metric_class,source_phase,source_frame,target_phase,target_frame,"
           "root_jump,heading_jump,velocity_jump,root_speed_ratio,yaw_rate_ratio,"
           "left_foot_drift,right_foot_drift,left_contact_before,left_contact_after,"
           "right_contact_before,right_contact_after,left_foot_height_before,"
           "left_foot_height_after,left_foot_height_delta,right_foot_height_before,"
           "right_foot_height_after,right_foot_height_delta,"
           "left_contact_mismatch,right_contact_mismatch,quality_gate_decision,"
           "reject_reason\n";
    const auto& q = row.quality;
    csv << ParameterCsv(row.source_parameter) << ','
        << ParameterCsv(row.requested_target_parameter) << ','
        << ParameterCsv(row.effective_target_parameter) << ','
        << ParameterCsv(row.target_box.min_corner) << ','
        << ParameterCsv(row.target_box.max_corner) << ','
        << (row.accepted_by_box ? "true" : "false") << ','
        << row.transition.distance << ',' << row.metric_class << ','
        << row.transition.source_phase << ',' << row.transition.source_frame << ','
        << row.transition.target_phase << ',' << row.transition.target_frame << ','
        << row.root_jump << ',' << row.heading_jump << ',' << row.velocity_jump << ','
        << q.root_speed_ratio << ',' << q.yaw_rate_ratio << ','
        << q.left_foot_drift << ',' << q.right_foot_drift << ','
        << pmg::TransitionContactStateName(q.left_contact_before) << ','
        << pmg::TransitionContactStateName(q.left_contact_after) << ','
        << pmg::TransitionContactStateName(q.right_contact_before) << ','
        << pmg::TransitionContactStateName(q.right_contact_after) << ','
        << q.left_foot_height_before << ',' << q.left_foot_height_after << ','
        << q.left_foot_height_delta << ',' << q.right_foot_height_before << ','
        << q.right_foot_height_after << ',' << q.right_foot_height_delta << ','
        << (ContactMismatch(q.left_contact_before, q.left_contact_after) ? "true" : "false") << ','
        << (ContactMismatch(q.right_contact_before, q.right_contact_after) ? "true" : "false") << ','
        << (row.final_accepted ? "accept" : "reject") << ','
        << row.reject_reason << '\n';
}

void WriteTransitionProbeMarkdown(
    const std::string& path,
    const TransitionProbeOptions& options,
    const TransitionProbeResult& row) {
    std::ofstream md(path);
    if (!md) throw std::runtime_error("--probe-transition: cannot write markdown");
    const auto& q = row.quality;
    md << "# Exact Transition Probe\n\n"
       << "- Artifact: `" << options.pmg_path << "`\n"
       << "- Source parameter: " << ParameterMd(row.source_parameter) << "\n"
       << "- Requested/effective target: "
       << ParameterMd(row.requested_target_parameter) << " / "
       << ParameterMd(row.effective_target_parameter) << "\n"
       << "- Target box: " << ParameterMd(row.target_box.min_corner) << " .. "
       << ParameterMd(row.target_box.max_corner) << "\n"
       << "- Box accepted: `" << (row.accepted_by_box ? "true" : "false") << "`\n"
       << "- D/class: " << row.transition.distance << " / `" << row.metric_class << "`\n"
       << "- Source/target phase (frame): " << row.transition.source_phase << " ("
       << row.transition.source_frame << ") / " << row.transition.target_phase << " ("
       << row.transition.target_frame << ")\n"
       << "- Root/heading/velocity jump: " << row.root_jump << " / "
       << row.heading_jump << " / " << row.velocity_jump << "\n"
       << "- Root speed/yaw rate ratio: " << q.root_speed_ratio << " / "
       << q.yaw_rate_ratio << "\n"
       << "- Left/right foot drift: " << q.left_foot_drift << " / "
       << q.right_foot_drift << "\n"
       << "- Left foot contact before/after, height before/after: `"
       << pmg::TransitionContactStateName(q.left_contact_before) << "` / `"
       << pmg::TransitionContactStateName(q.left_contact_after) << "`, "
       << q.left_foot_height_before << " / " << q.left_foot_height_after << "\n"
       << "- Right foot contact before/after, height before/after: `"
       << pmg::TransitionContactStateName(q.right_contact_before) << "` / `"
       << pmg::TransitionContactStateName(q.right_contact_after) << "`, "
       << q.right_foot_height_before << " / " << q.right_foot_height_after << "\n"
       << "- Quality decision: `" << (row.final_accepted ? "accept" : "reject")
       << "`\n- Reject reason: `" << row.reject_reason << "`\n";
}

TransitionMontageAuditOptions ParseTransitionMontageAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-transition-montage needs <graph.pmg>");
    }
    TransitionMontageAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--output-dir") {
            options.output_dir = require_value("--output-dir");
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--samples") {
            options.samples_per_axis = std::stoi(require_value("--samples"));
        } else {
            throw std::runtime_error(
                "unknown audit-transition-montage option '" + option + "'");
        }
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-transition-montage requires --output-md");
    }
    if (options.output_dir.empty()) {
        throw std::runtime_error(
            "--audit-transition-montage requires --output-dir");
    }
    if (options.samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-transition-montage: --samples must be at least 2");
    }
    return options;
}

TransitionProbeOptions ParseTransitionProbeOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--probe-transition needs <graph.pmg>");
    }
    TransitionProbeOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            return argv[++index];
        };
        auto require_parameter_2d = [&](const char* name) {
            const float x = std::stof(require_value(name));
            const float y = std::stof(require_value(name));
            return pmg::ParameterVector{x, y};
        };
        if (option == "--source-param") {
            options.source_parameter = require_parameter_2d("--source-param");
        } else if (option == "--target-param") {
            options.target_parameter = require_parameter_2d("--target-param");
        } else if (option == "--frames-before") {
            options.frames_before = std::stoi(require_value("--frames-before"));
        } else if (option == "--frames-after") {
            options.frames_after = std::stoi(require_value("--frames-after"));
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--expect-decision") {
            options.expected_decision = require_value("--expect-decision");
        } else if (option == "--max-root-speed-ratio") {
            options.quality_gate.max_root_speed_ratio =
                std::stof(require_value("--max-root-speed-ratio"));
        } else if (option == "--max-yaw-rate-ratio") {
            options.quality_gate.max_yaw_rate_ratio =
                std::stof(require_value("--max-yaw-rate-ratio"));
        } else if (option == "--max-contact-drift") {
            options.quality_gate.max_contact_drift =
                std::stof(require_value("--max-contact-drift"));
        } else if (option == "--max-foot-height-delta") {
            options.quality_gate.max_foot_height_delta =
                std::stof(require_value("--max-foot-height-delta"));
        } else if (option == "--allow-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = false;
        } else if (option == "--reject-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = true;
        } else {
            throw std::runtime_error(
                "unknown probe-transition option '" + option + "'");
        }
    }
    if (options.source_parameter.empty() || options.target_parameter.empty()) {
        throw std::runtime_error(
            "--probe-transition requires --source-param and --target-param");
    }
    if (options.frames_before < 1 || options.frames_after < 1) {
        throw std::runtime_error(
            "--probe-transition frame counts must be positive");
    }
    if (!options.expected_decision.empty() &&
        options.expected_decision != "accept" &&
        options.expected_decision != "reject") {
        throw std::runtime_error(
            "--expect-decision must be accept or reject");
    }
    return options;
}

ContactTransitionAuditOptions ParseContactTransitionAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-contact-transitions needs <graph.pmg>");
    }
    ContactTransitionAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        if (index + 1 >= argc) {
            throw std::runtime_error(option + " requires a value");
        }
        const std::string value = argv[++index];
        if (option == "--output-csv") options.output_csv = value;
        else if (option == "--output-md") options.output_md = value;
        else if (option == "--source-node") options.source_node = value;
        else if (option == "--target-node") options.target_node = value;
        else if (option == "--samples") options.samples_per_axis = std::stoi(value);
        else throw std::runtime_error(
            "unknown audit-contact-transitions option '" + option + "'");
    }
    if (options.output_csv.empty() || options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-contact-transitions requires --output-csv and --output-md");
    }
    if (options.samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-contact-transitions: --samples must be at least 2");
    }
    return options;
}

TransitionAcceptanceConsistencyAuditOptions
ParseTransitionAcceptanceConsistencyAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency needs <graph.pmg>");
    }
    TransitionAcceptanceConsistencyAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--samples") {
            options.samples_per_axis = std::stoi(require_value("--samples"));
        } else if (option == "--max-root-speed-ratio") {
            options.quality_gate.max_root_speed_ratio =
                std::stof(require_value("--max-root-speed-ratio"));
        } else if (option == "--max-yaw-rate-ratio") {
            options.quality_gate.max_yaw_rate_ratio =
                std::stof(require_value("--max-yaw-rate-ratio"));
        } else if (option == "--max-contact-drift") {
            options.quality_gate.max_contact_drift =
                std::stof(require_value("--max-contact-drift"));
        } else if (option == "--max-foot-height-delta") {
            options.quality_gate.max_foot_height_delta =
                std::stof(require_value("--max-foot-height-delta"));
        } else if (option == "--allow-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = false;
        } else if (option == "--reject-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = true;
        } else {
            throw std::runtime_error(
                "unknown audit-transition-acceptance-consistency option '" +
                option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency requires --output-md");
    }
    if (options.samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: --samples must be at least 2");
    }
    return options;
}

SkeletonAuditOptions ParseSkeletonAuditOptions(int argc, char** argv) {
    if (argc < 5) {
        throw std::runtime_error(
            "usage: pmg_cli inspect-skeleton input.bvh --out outputs/skeleton_report");
    }
    SkeletonAuditOptions options;
    options.bvh_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        if (option != "--out") {
            throw std::runtime_error("inspect-skeleton: unknown option '" + option + "'");
        }
        if (index + 1 >= argc) {
            throw std::runtime_error("inspect-skeleton: --out requires a value");
        }
        options.out_dir = argv[++index];
    }
    if (options.out_dir.empty()) {
        throw std::runtime_error("inspect-skeleton requires --out");
    }
    return options;
}

LoopAuditOptions ParseLoopAuditOptions(int argc, char** argv) {
    if (argc < 5) {
        throw std::runtime_error(
            "usage: pmg_cli audit-loop input.bvh --out outputs/loop_audit");
    }
    LoopAuditOptions options;
    options.bvh_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            return argv[++index];
        };
        if (option == "--out") {
            options.out_dir = require_value("--out");
        } else if (option == "--start-frame") {
            options.start_frame = std::stoi(require_value("--start-frame"));
        } else if (option == "--end-frame") {
            options.end_frame = std::stoi(require_value("--end-frame"));
        } else {
            throw std::runtime_error("audit-loop: unknown option '" + option + "'");
        }
    }
    if (options.out_dir.empty()) {
        throw std::runtime_error("audit-loop requires --out");
    }
    return options;
}

TransitionPopAuditOptions ParseTransitionPopAuditOptions(int argc, char** argv) {
    if (argc < 5) {
        throw std::runtime_error(
            "usage: pmg_cli audit-transition-pop artifact.pmg --out outputs/transition_pop_audit");
    }
    TransitionPopAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            return argv[++index];
        };
        if (option == "--out") {
            options.out_dir = require_value("--out");
        } else if (option == "--worst-k") {
            options.worst_k = std::stoi(require_value("--worst-k"));
        } else {
            throw std::runtime_error(
                "audit-transition-pop: unknown option '" + option + "'");
        }
    }
    if (options.out_dir.empty()) {
        throw std::runtime_error("audit-transition-pop requires --out");
    }
    if (options.worst_k < 1) {
        throw std::runtime_error("audit-transition-pop: --worst-k must be positive");
    }
    return options;
}

int TransitionMontageAuditCommand(
    const TransitionMontageAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const TransitionMontageAuditData data =
        BuildTransitionMontageAudit(artifact, options);
    WriteTransitionMontageManifestCsv(data);
    WriteTransitionMontageMarkdown(options, data);
    std::cout << "transition_montage_rows=" << data.rows.size() << "\n";
    std::cout << "transition_montage_md=" << options.output_md << "\n";
    std::cout << "transition_montage_manifest=" << data.manifest_csv_path
              << "\n";
    std::cout << "transition_montage_conclusion=" << data.conclusion << "\n";
    return 0;
}

int SkeletonAuditCommand(const SkeletonAuditOptions& options) {
    const pmg::BvhData data = pmg::BvhLoader::Load(options.bvh_path);
    const pmg::SkeletonInspectionReport report =
        pmg::InspectSkeleton(data, options.bvh_path);
    pmg::WriteSkeletonInspectionArtifacts(report, options.out_dir);
    std::cout << "skeleton_report=" << (options.out_dir / "skeleton_report.md").string()
              << "\n";
    return 0;
}

int LoopAuditCommand(const LoopAuditOptions& options) {
    const pmg::BvhData data = pmg::BvhLoader::Load(options.bvh_path);
    pmg::LoopAuditReport report =
        pmg::AuditLoop(data, options.start_frame, options.end_frame);
    report.source_path = options.bvh_path;
    const std::filesystem::path out_path = options.out_dir / "loop_audit.md";
    pmg::WriteLoopAuditReport(report, out_path);
    std::cout << "loop_audit=" << out_path.string() << "\n";

    // Per-joint contact-interval timeline over the full clip. Lets an author
    // pick a phase-aligned segment start (e.g. a left-foot strike frame) when
    // windowing a MotionClipSegment example.
    std::vector<int> contact_joints;
    std::vector<std::string> contact_joint_names;
    for (const char* joint_name : {"LeftAnkle", "RightAnkle", "LeftFoot",
                                   "RightFoot", "LeftToeBase", "RightToeBase"}) {
        for (int j = 0; j < data.skeleton.NumJoints(); ++j) {
            if (data.skeleton.joints[j].name == joint_name) {
                contact_joints.push_back(j);
                contact_joint_names.push_back(joint_name);
                break;
            }
        }
    }
    if (!contact_joints.empty()) {
        const pmg::ContactDetectionSettings settings =
            pmg::EstimateContactSettings(data.skeleton, data.clip, contact_joints);
        const std::vector<pmg::ContactInterval> intervals =
            pmg::DetectContacts(data.skeleton, data.clip, contact_joints, settings);
        const int frame_count = data.clip.NumFrames();
        const std::filesystem::path timeline_path =
            options.out_dir / "contact_timeline.csv";
        std::ofstream timeline(timeline_path);
        if (!timeline) {
            throw std::runtime_error(
                "audit-loop: cannot write contact_timeline.csv");
        }
        timeline << "joint,first_frame,last_frame,strike_phase,lift_phase\n";
        for (const pmg::ContactInterval& interval : intervals) {
            std::string joint_label = std::to_string(interval.joint_index);
            for (std::size_t i = 0; i < contact_joints.size(); ++i) {
                if (contact_joints[i] == interval.joint_index) {
                    joint_label = contact_joint_names[i];
                    break;
                }
            }
            timeline << joint_label << ',' << interval.first_frame << ','
                     << interval.last_frame << ','
                     << interval.StrikePhase(frame_count) << ','
                     << interval.LiftPhase(frame_count) << "\n";
        }
        std::cout << "contact_timeline=" << timeline_path.string() << "\n";
    }
    return 0;
}

int TransitionPopAuditCommand(const TransitionPopAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact = pmg::LoadPmgArtifactText(options.pmg_path);
    const pmg::TransitionPopAuditReport report =
        pmg::AuditTransitionPop(artifact, options.worst_k);
    std::filesystem::create_directories(options.out_dir);
    const std::filesystem::path out_path =
        options.out_dir / "transition_pop_audit.md";
    const std::filesystem::path worst_csv_path =
        options.out_dir / "worst_aligned_transitions.csv";
    {
        std::ofstream csv(worst_csv_path);
        if (!csv) {
            throw std::runtime_error(
                "audit-transition-pop: cannot write worst_aligned_transitions.csv");
        }
        csv << "source_node,target_node,source_parameter,target_parameter,"
               "source_transition_frame,target_transition_frame,"
               "stored_transition_distance,raw_root_jump,aligned_root_jump,"
               "raw_joint_jump,aligned_joint_jump,raw_velocity_jump,"
               "aligned_velocity_jump,contact_mismatch,alignment_yaw,"
               "alignment_translation_x,alignment_translation_z\n";
        for (const pmg::TransitionPopAuditRow& row : report.worst_transitions) {
            csv << EscapeCsvCell(row.source_node) << ','
                << EscapeCsvCell(row.target_node) << ','
                << ParameterCsv(row.source_parameter) << ','
                << ParameterCsv(row.target_parameter) << ','
                << row.source_frame << ','
                << row.target_frame << ','
                << row.transition_distance << ','
                << row.raw_root_position_jump << ','
                << row.aligned_root_position_jump << ','
                << row.raw_joint_pose_jump << ','
                << row.aligned_joint_pose_jump << ','
                << row.raw_velocity_jump << ','
                << row.aligned_velocity_jump << ','
                << row.contact_mismatch << ','
                << row.alignment_yaw << ','
                << row.alignment_dx << ','
                << row.alignment_dz << "\n";
        }
    }

    const std::vector<ArtifactLoopSweepRow> loop_rows =
        BuildArtifactLoopSweepRows(artifact);
    WriteArtifactLoopSweepCsv(
        options.out_dir / "generated_loop_sweep.csv", loop_rows);
    WriteArtifactLoopSweepMarkdown(
        options.out_dir / "generated_loop_sweep.md", loop_rows);

    pmg::WriteTransitionPopAuditReport(report, out_path);
    std::cout << "transition_pop_audit=" << out_path.string() << "\n";
    std::cout << "transition_pop_worst_csv=" << worst_csv_path.string() << "\n";
    std::cout << "generated_loop_sweep_csv="
              << (options.out_dir / "generated_loop_sweep.csv").string() << "\n";
    std::cout << "generated_loop_sweep_md="
              << (options.out_dir / "generated_loop_sweep.md").string() << "\n";
    return 0;
}

int ContactTransitionAuditCommand(
    const ContactTransitionAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const auto rows = BuildContactTransitionRows(artifact, options);
    WriteContactTransitionAudit(options, rows);
    std::cout << "contact_transition_rows=" << rows.size() << "\n";
    std::cout << "contact_transition_conclusion="
              << ContactTransitionConclusion(rows) << "\n";
    return 0;
}

int TransitionProbeCommand(const TransitionProbeOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const TransitionProbeResult row = BuildTransitionProbe(artifact, options);
    if (!options.output_csv.empty()) WriteTransitionProbeCsv(options.output_csv, row);
    if (!options.output_md.empty()) {
        WriteTransitionProbeMarkdown(options.output_md, options, row);
    }
    const auto& q = row.quality;
    std::cout << std::setprecision(9)
              << "source_parameter=" << ParameterMd(row.source_parameter) << '\n'
              << "requested_target_parameter="
              << ParameterMd(row.requested_target_parameter) << '\n'
              << "effective_target_parameter="
              << ParameterMd(row.effective_target_parameter) << '\n'
              << "target_box_min=" << ParameterMd(row.target_box.min_corner) << '\n'
              << "target_box_max=" << ParameterMd(row.target_box.max_corner) << '\n'
              << "accepted_by_box=" << (row.accepted_by_box ? "true" : "false") << '\n'
              << "transition_metric_d=" << row.transition.distance << '\n'
              << "metric_class=" << row.metric_class << '\n'
              << "source_phase=" << row.transition.source_phase << '\n'
              << "source_frame=" << row.transition.source_frame << '\n'
              << "target_phase=" << row.transition.target_phase << '\n'
              << "target_frame=" << row.transition.target_frame << '\n'
              << "root_jump=" << row.root_jump << '\n'
              << "heading_jump=" << row.heading_jump << '\n'
              << "velocity_jump=" << row.velocity_jump << '\n'
              << "root_speed_ratio=" << q.root_speed_ratio << '\n'
              << "yaw_rate_ratio=" << q.yaw_rate_ratio << '\n'
              << "left_foot_drift=" << q.left_foot_drift << '\n'
              << "right_foot_drift=" << q.right_foot_drift << '\n'
              << "left_contact_before="
              << pmg::TransitionContactStateName(q.left_contact_before) << '\n'
              << "left_contact_after="
              << pmg::TransitionContactStateName(q.left_contact_after) << '\n'
              << "right_contact_before="
              << pmg::TransitionContactStateName(q.right_contact_before) << '\n'
              << "right_contact_after="
              << pmg::TransitionContactStateName(q.right_contact_after) << '\n'
              << "left_contact_mismatch="
              << (ContactMismatch(q.left_contact_before, q.left_contact_after)
                      ? "true" : "false") << '\n'
              << "right_contact_mismatch="
              << (ContactMismatch(q.right_contact_before, q.right_contact_after)
                      ? "true" : "false") << '\n'
              << "left_foot_height_before=" << q.left_foot_height_before << '\n'
              << "left_foot_height_after=" << q.left_foot_height_after << '\n'
              << "left_foot_height_delta=" << q.left_foot_height_delta << '\n'
              << "right_foot_height_before=" << q.right_foot_height_before << '\n'
              << "right_foot_height_after=" << q.right_foot_height_after << '\n'
              << "right_foot_height_delta=" << q.right_foot_height_delta << '\n'
              << "final_quality_gate_decision="
              << (row.final_accepted ? "accept" : "reject") << '\n'
              << "reject_reason=" << row.reject_reason << '\n';
    const std::string actual_decision = row.final_accepted ? "accept" : "reject";
    return options.expected_decision.empty() ||
                   options.expected_decision == actual_decision
               ? 0
               : 1;
}

int TransitionAcceptanceConsistencyAuditCommand(
    const TransitionAcceptanceConsistencyAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const TransitionAcceptanceConsistencyAuditData data =
        BuildTransitionAcceptanceConsistencyAudit(artifact, options);
    WriteTransitionAcceptanceConsistencyCsv(options, data);
    WriteTransitionAcceptanceConsistencyMarkdown(options, data);
    std::cout << "transition_acceptance_consistency_rows="
              << data.rows.size() << "\n";
    std::cout << "transition_acceptance_consistency_csv="
              << options.output_csv << "\n";
    std::cout << "transition_acceptance_consistency_md="
              << options.output_md << "\n";
    std::cout << "transition_acceptance_consistency_conclusion="
              << data.consistency_conclusion << "\n";
    std::cout << "transition_acceptance_consistency_root_cause="
              << data.root_cause_conclusion << "\n";
    std::cout << "accepted_bad_by_box=" << data.accepted_bad_count << "\n";
    std::cout << "accepted_bad_after_quality_gate="
              << data.accepted_bad_after_quality_gate << "\n";
    std::cout << "box_accepted_quality_rejected="
              << data.box_accepted_quality_rejected << "\n";
    return data.accepted_bad_after_quality_gate == 0 ? 0 : 1;
}

pmg::PmgBuilderConfig EffectiveEdgeConfig(
    const pmg::GraphSpecEdge& edge,
    const ValidateGraphOptions& options) {
    pmg::PmgBuilderConfig config =
        edge.has_build_config ? edge.build_config : options.builder;

    // Explicit CLI values are diagnostic overrides. Unspecified values retain
    // the authored per-edge config so validation matches artifact construction.
    if (options.tgood_set) {
        config.good_transition_threshold =
            options.builder.good_transition_threshold;
    }
    if (options.tbad_set) {
        config.bad_transition_threshold =
            options.builder.bad_transition_threshold;
    }
    if (options.source_samples_set) {
        config.source_sample_count = options.builder.source_sample_count;
    }
    if (options.target_samples_set) {
        config.target_sample_count = options.builder.target_sample_count;
    }
    if (options.seed_set) {
        config.seed = options.builder.seed;
    }
    return config;
}

void PrintEffectiveEdgeConfig(const pmg::PmgBuilderConfig& config) {
    std::cout << "effective_edge_config:"
              << " TGOOD=" << config.good_transition_threshold
              << " TBAD=" << config.bad_transition_threshold
              << " source_samples=" << config.source_sample_count
              << " target_samples=" << config.target_sample_count
              << " window_size=" << config.distance_grid.window_size
              << " convention="
              << pmg::TransitionWindowConventionName(
                     config.transition_convention)
              << "\n";
}

struct EdgeQuality {
    bool created = false;
    int samples = 0;
    float mean_good_fraction = 0.0f;
    float mean_min_distance = 0.0f;
    float mean_p25_distance = 0.0f;
    float mean_median_distance = 0.0f;
    float mean_box_volume_fraction = 0.0f;
};

EdgeQuality MeasureEdgeQuality(
    const pmg::EdgeBuildResult& result,
    const pmg::ParametricMotionSpace& target_space) {
    EdgeQuality quality;
    quality.created = result.report.edge_created;
    quality.samples = static_cast<int>(result.edge.samples.size());

    float good_fraction_sum = 0.0f;
    float min_distance_sum = 0.0f;
    float p25_distance_sum = 0.0f;
    float median_distance_sum = 0.0f;
    int report_count = 0;
    for (const pmg::SourceSampleBuildReport& report : result.report.source_reports) {
        const int total = report.good_count + report.neutral_count + report.bad_count;
        if (total > 0) {
            good_fraction_sum += static_cast<float>(report.good_count) / static_cast<float>(total);
        }
        min_distance_sum += report.min_distance;
        p25_distance_sum += report.p25_distance;
        median_distance_sum += report.median_distance;
        ++report_count;
    }
    if (report_count > 0) {
        quality.mean_good_fraction = good_fraction_sum / static_cast<float>(report_count);
        quality.mean_min_distance = min_distance_sum / static_cast<float>(report_count);
        quality.mean_p25_distance = p25_distance_sum / static_cast<float>(report_count);
        quality.mean_median_distance = median_distance_sum / static_cast<float>(report_count);
    }

    const std::vector<float> domain_min = target_space.MinParameter();
    const std::vector<float> domain_max = target_space.MaxParameter();
    float volume_sum = 0.0f;
    for (const pmg::TransitionSample& sample : result.edge.samples) {
        float volume = 1.0f;
        for (std::size_t dim = 0; dim < domain_min.size(); ++dim) {
            const float domain_extent = std::max(domain_max[dim] - domain_min[dim], 1.0e-6f);
            const float box_extent =
                sample.target_parameter_box.max_corner[dim] -
                sample.target_parameter_box.min_corner[dim];
            volume *= std::clamp(box_extent / domain_extent, 0.0f, 1.0f);
        }
        volume_sum += volume;
    }
    if (quality.samples > 0) {
        quality.mean_box_volume_fraction = volume_sum / static_cast<float>(quality.samples);
    }
    return quality;
}

void PrintEdgeQuality(const char* label, const EdgeQuality& quality) {
    std::cout << label << "_edge_created=" << (quality.created ? 1 : 0) << "\n";
    std::cout << label << "_samples=" << quality.samples << "\n";
    std::cout << label << "_mean_good_fraction=" << quality.mean_good_fraction << "\n";
    std::cout << label << "_mean_min_distance=" << quality.mean_min_distance << "\n";
    std::cout << label << "_mean_p25_distance=" << quality.mean_p25_distance << "\n";
    std::cout << label << "_mean_median_distance=" << quality.mean_median_distance << "\n";
    std::cout << label << "_mean_box_volume_fraction=" << quality.mean_box_volume_fraction << "\n";
}

int ValidateGraphCommand(const ValidateGraphOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    pmg::MotionSpacePreparationConfig preparation_config;
    preparation_config.default_cycle_joint = options.cycle_joint;
    preparation_config.default_contact_joints =
        SplitCommaList(options.contact_joints_csv);
    preparation_config.default_min_contact_frames =
        options.min_contact_frames;
    preparation_config.default_dtw_refine = true;
    preparation_config.calibration_frames_per_second =
        options.builder.generated_frames_per_second;
    const pmg::PreparedMotionSpaces prepared =
        pmg::PrepareMotionSpaces(spec, preparation_config);

    // Compare authored inputs against the exact production motion spaces
    // consumed by artifact edge construction.
    std::map<std::string, pmg::ParametricMotionSpace> naive_spaces;
    std::map<std::string, pmg::ParametricMotionSpace> production_spaces;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        const pmg::PreparedMotionSpace& prepared_node =
            prepared.Node(node.name);
        RequireMatchingProductionRegistration(
            prepared_node.registration, node.name, options.cycle_joint,
            options.cycle_joint_set,
            SplitCommaList(options.contact_joints_csv),
            options.contact_joints_set, options.min_contact_frames,
            options.min_contact_frames_set, "--validate-graph");
        naive_spaces.emplace(node.name, prepared_node.authored);
        production_spaces.emplace(node.name, prepared_node.production);
    }

    bool failed = false;
    auto fail_if = [&failed](bool condition, const std::string& message) {
        if (condition) {
            std::cout << "ASSERT FAIL: " << message << "\n";
            failed = true;
        }
    };

    for (const pmg::GraphSpecEdge& edge : spec.edges) {
        std::cout << "=== edge " << edge.source_node << " -> " << edge.target_node << " ===\n";
        const pmg::PmgBuilderConfig edge_config =
            EffectiveEdgeConfig(edge, options);
        PrintEffectiveEdgeConfig(edge_config);

        const pmg::EdgeBuildResult naive_result = pmg::PmgBuilder::BuildEdgeWithReport(
            prepared.skeleton, 0, 0, naive_spaces.at(edge.source_node),
            naive_spaces.at(edge.target_node), edge_config);
        const pmg::EdgeBuildResult production_result =
            pmg::PmgBuilder::BuildEdgeWithReport(
                prepared.skeleton, 0, 0,
                production_spaces.at(edge.source_node),
                production_spaces.at(edge.target_node), edge_config);

        const EdgeQuality naive = MeasureEdgeQuality(
            naive_result, naive_spaces.at(edge.target_node));
        const EdgeQuality production = MeasureEdgeQuality(
            production_result, production_spaces.at(edge.target_node));

        PrintEdgeQuality("naive", naive);
        PrintEdgeQuality("production", production);
        if (!production_result.report.edge_created) {
            std::cout << "production_reject_reason="
                      << production_result.report.reject_reason
                      << "\n";
        }

        fail_if(!production.created, "production edge not created");
        if (options.min_edge_samples >= 0) {
            fail_if(production.samples < options.min_edge_samples,
                    "production_samples=" + std::to_string(production.samples) + " < " +
                        std::to_string(options.min_edge_samples));
        }
        if (options.min_good_fraction >= 0.0f) {
            fail_if(production.mean_good_fraction < options.min_good_fraction,
                    "production_mean_good_fraction=" +
                        std::to_string(production.mean_good_fraction) + " < " +
                        std::to_string(options.min_good_fraction));
        }
        // Distribution comparisons are only meaningful when both builds
        // covered the same source samples (a rejected build aborts early).
        if (options.assert_no_regression) {
            fail_if(!naive.created || !production.created,
                    "no-regression comparison needs both builds to complete; "
                    "loosen --tgood/--tbad");
            if (naive.created && production.created) {
                fail_if(production.mean_good_fraction < naive.mean_good_fraction * 0.95f,
                        "production preparation shrank GOOD fraction: " +
                            std::to_string(production.mean_good_fraction) + " < " +
                            std::to_string(naive.mean_good_fraction));
            }
        }
        float max_distance_ratio =
            options.max_preparation_min_distance_ratio;
        if (options.assert_no_regression && max_distance_ratio < 0.0f) {
            max_distance_ratio = 1.05f;
        }
        if (max_distance_ratio >= 0.0f) {
            fail_if(!naive.created || !production.created,
                    "preparation-distance comparison needs both builds to complete; "
                    "loosen --tgood/--tbad");
            if (naive.created && production.created) {
                // Gate only on the BEST transition per source sample: that is
                // what the runtime schedules. Median/p25 remain report-only.
                fail_if(
                    production.mean_min_distance >
                        naive.mean_min_distance * max_distance_ratio,
                    "production/naive mean-min-distance ratio exceeded " +
                        std::to_string(max_distance_ratio) + ": " +
                        std::to_string(production.mean_min_distance) + " / " +
                        std::to_string(naive.mean_min_distance));
            }
        }
    }

    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

ValidateGraphOptions ParseValidateGraphOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--validate-graph needs <spec>");
    }
    ValidateGraphOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
            options.contact_joints_set = true;
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
            options.cycle_joint_set = true;
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames = std::stoi(require_value("--min-contact-frames"));
            options.min_contact_frames_set = true;
        } else if (option == "--tgood") {
            options.builder.good_transition_threshold = std::stof(require_value("--tgood"));
            options.tgood_set = true;
        } else if (option == "--tbad") {
            options.builder.bad_transition_threshold = std::stof(require_value("--tbad"));
            options.tbad_set = true;
        } else if (option == "--source-samples") {
            options.builder.source_sample_count = std::stoi(require_value("--source-samples"));
            options.source_samples_set = true;
        } else if (option == "--target-samples") {
            options.builder.target_sample_count = std::stoi(require_value("--target-samples"));
            options.target_samples_set = true;
        } else if (option == "--seed") {
            options.builder.seed =
                static_cast<unsigned int>(std::stoul(require_value("--seed")));
            options.seed_set = true;
        } else if (option == "--min-edge-samples") {
            options.min_edge_samples = std::stoi(require_value("--min-edge-samples"));
        } else if (option == "--min-good-fraction") {
            options.min_good_fraction = std::stof(require_value("--min-good-fraction"));
        } else if (option == "--assert-no-regression") {
            options.assert_no_regression = true;
        } else if (option == "--max-preparation-distance-ratio") {
            options.max_preparation_min_distance_ratio =
                std::stof(require_value("--max-preparation-distance-ratio"));
            if (!std::isfinite(options.max_preparation_min_distance_ratio) ||
                options.max_preparation_min_distance_ratio <= 0.0f) {
                throw std::runtime_error(
                    "--max-preparation-distance-ratio must be positive");
            }
        } else {
            throw std::runtime_error("unknown validate-graph option '" + option + "'");
        }
    }
    return options;
}

CyclicAuditOptions ParseCyclicAuditOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--audit-cyclic-continuity needs <spec>");
    }
    CyclicAuditOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(
                    std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--fps") {
            options.frames_per_second = std::stof(require_value("--fps"));
        } else {
            throw std::runtime_error(
                "unknown audit-cyclic-continuity option '" + option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-cyclic-continuity requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-cyclic-continuity requires --output-md");
    }
    if (options.frames_per_second <= 0.0f) {
        throw std::runtime_error("--fps must be positive");
    }
    return options;
}

}  // namespace

namespace pmgcli {

std::optional<int> TryRunDiagnosticCommand(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "--validate-graph" && argc >= 3) {
        return ValidateGraphCommand(ParseValidateGraphOptions(argc, argv));
    }
    if (command == "--audit-cyclic-continuity" && argc >= 3) {
        return CyclicAuditCommand(
            ParseCyclicAuditOptions(argc, argv),
            CommandLineString(argc, argv));
    }
    if (command == "--audit-transition-montage" && argc >= 3) {
        return TransitionMontageAuditCommand(
            ParseTransitionMontageAuditOptions(argc, argv));
    }
    if ((command == "audit-loop" || command == "--audit-loop") && argc >= 5) {
        return LoopAuditCommand(ParseLoopAuditOptions(argc, argv));
    }
    if ((command == "audit-transition-pop" || command == "--audit-transition-pop") &&
        argc >= 5) {
        return TransitionPopAuditCommand(ParseTransitionPopAuditOptions(argc, argv));
    }
    if ((command == "audit-motion-space-registration" ||
         command == "--audit-motion-space-registration") &&
        argc >= 5) {
        return MotionSpaceRegistrationAuditCommand(
            ParseMotionSpaceRegistrationAuditOptions(argc, argv));
    }
    if ((command == "inspect-skeleton" || command == "--inspect-skeleton") &&
        argc >= 5) {
        return SkeletonAuditCommand(ParseSkeletonAuditOptions(argc, argv));
    }
    if (command == "--probe-transition" && argc >= 3) {
        return TransitionProbeCommand(ParseTransitionProbeOptions(argc, argv));
    }
    if (command == "--audit-contact-transitions" && argc >= 3) {
        return ContactTransitionAuditCommand(
            ParseContactTransitionAuditOptions(argc, argv));
    }
    if (command == "--audit-transition-acceptance-consistency" && argc >= 3) {
        return TransitionAcceptanceConsistencyAuditCommand(
            ParseTransitionAcceptanceConsistencyAuditOptions(argc, argv));
    }
    return std::nullopt;
}

}  // namespace pmgcli
