#include "pmg/MotionSpacePreparation.h"

#include "pmg/BvhLoader.h"
#include "pmg/MotionRegistration.h"
#include "pmg/SkeletonCompatibility.h"

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

int FindJointIndex(const Skeleton& skeleton, const std::string& name, const std::string& context) {
    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        if (skeleton.joints[joint_index].name == name) {
            return joint_index;
        }
    }
    throw std::runtime_error(context + ": unknown joint '" + name + "'");
}

bool IsExplicitRecutBvhPath(const std::string& path) {
    // Files produced by `pmg_cli --export-known-cyclic-recuts` are already
    // cropped to the intended cycle window. Running ExtractFirstCycle again on
    // those short clips can fail because a valid one-cycle clip may contain only
    // one detected strike for the cycle joint. Keep registration/contact metadata
    // active, but do not perform a second contact-based cycle extraction.
    const std::string filename = std::filesystem::path(path).filename().string();
    return filename.find("_recut_") != std::string::npos;
}

NodeRegistrationMetadata ResolveRegistration(const GraphSpecNode& node,
                                             const MotionSpacePreparationConfig& config) {
    NodeRegistrationMetadata registration;
    registration.node_name = node.name;
    registration.cycle_joint =
        node.has_registration_config ? node.cycle_joint : config.default_cycle_joint;
    registration.contact_joints =
        node.has_registration_config ? node.contact_joints : config.default_contact_joints;
    registration.min_contact_frames =
        node.has_registration_config ? node.min_contact_frames : config.default_min_contact_frames;
    registration.dtw_refine =
        node.has_registration_config ? node.dtw_refine : config.default_dtw_refine;
    return registration;
}

}  // namespace

const PreparedMotionSpace& PreparedMotionSpaces::Node(const std::string& name) const {
    const auto node = nodes.find(name);
    if (node == nodes.end()) {
        throw std::runtime_error("PreparedMotionSpaces: unknown node '" + name + "'");
    }
    return node->second;
}

PreparedMotionSpaces PrepareMotionSpaces(const GraphSpec& spec,
                                         const MotionSpacePreparationConfig& config) {
    if (spec.nodes.empty()) {
        throw std::runtime_error("PrepareMotionSpaces: spec contains no nodes");
    }
    if (config.default_min_contact_frames <= 0) {
        throw std::runtime_error(
            "PrepareMotionSpaces: default_min_contact_frames must be positive");
    }
    if (config.calibration_frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "PrepareMotionSpaces: calibration_frames_per_second must be positive");
    }

    PreparedMotionSpaces prepared;
    std::optional<Skeleton> reference_skeleton;

    for (const GraphSpecNode& node : spec.nodes) {
        PreparedMotionSpace stages;
        stages.registration = ResolveRegistration(node, config);
        ParametricMotionSpace authored(node.name, node.parameter_dimension);

        for (const GraphSpecExample& example : spec.examples) {
            if (example.node_name != node.name) {
                continue;
            }

            BvhData bvh = BvhLoader::Load(example.bvh_path);
            if (!reference_skeleton.has_value()) {
                reference_skeleton = bvh.skeleton;
            } else {
                RequireSkeletonCompatible(*reference_skeleton, bvh.skeleton, "PrepareMotionSpaces",
                                          config.skeleton_offset_tolerance);
            }

            MotionClip clip = std::move(bvh.clip);
            const bool should_extract_first_cycle =
                !stages.registration.cycle_joint.empty() &&
                !IsExplicitRecutBvhPath(example.bvh_path);
            if (should_extract_first_cycle) {
                const int cycle_joint =
                    FindJointIndex(*reference_skeleton, stages.registration.cycle_joint,
                                   "PrepareMotionSpaces cycle joint");
                const ContactDetectionSettings cycle_settings =
                    EstimateContactSettings(*reference_skeleton, clip, {cycle_joint});
                clip = ExtractFirstCycle(*reference_skeleton, clip, cycle_joint, cycle_settings);
            }

            authored.AddExample(example.parameter, std::move(clip));
            prepared.source_bvh_paths.push_back(example.bvh_path);
        }

        if (authored.NumExamples() == 0) {
            throw std::runtime_error("PrepareMotionSpaces: node '" + node.name +
                                     "' has no examples");
        }

        if (node.parameter_support_simplex) {
            authored.SetParameterSupport(ParameterSupport(authored.ExampleParameters()));
        } else if (node.parameter_support_triangulated_2d) {
            authored.SetParameterSupport(ParameterSupport::CreateTriangulated2D(
                authored.ExampleParameters(), node.parameter_triangles));
        }

        stages.authored = authored;
        ParametricMotionSpace production = authored;

        if (!stages.registration.contact_joints.empty()) {
            for (const std::string& joint_name : stages.registration.contact_joints) {
                stages.contact_joint_indices.push_back(FindJointIndex(
                    *reference_skeleton, joint_name, "PrepareMotionSpaces contact joint"));
            }

            ContactDetectionSettings settings =
                EstimateContactSettings(*reference_skeleton, production.Examples().front().clip,
                                        stages.contact_joint_indices);
            settings.min_contact_frames = stages.registration.min_contact_frames;
            stages.contact_settings = settings;

            // Reject cross-family blend spaces before registration: every
            // example must be one looping cycle sharing one contact structure.
            std::vector<std::vector<ContactInterval>> example_intervals;
            std::vector<int> example_frame_counts;
            example_intervals.reserve(production.Examples().size());
            example_frame_counts.reserve(production.Examples().size());
            for (const ExampleMotion& example : production.Examples()) {
                example_intervals.push_back(DetectContacts(
                    *reference_skeleton, example.clip, stages.contact_joint_indices,
                    settings));
                example_frame_counts.push_back(example.clip.NumFrames());
            }
            RequireBlendableMotionFamily(example_intervals, example_frame_counts);

            RegisterSpaceByContacts(production, *reference_skeleton, stages.contact_joint_indices,
                                    settings);
            stages.contact_registered = production;

            if (stages.registration.dtw_refine) {
                RefineRegistrationByDtw(production, *reference_skeleton, {});
                stages.dtw_refined = production;
            }
        } else if (stages.registration.dtw_refine) {
            throw std::runtime_error("PrepareMotionSpaces: node '" + node.name +
                                     "' enables DTW refinement without contact joints");
        }

        if (!node.parameter_metrics.empty()) {
            production.SetParameterCalibration(CalibrateParameterMetrics(
                production, node.parameter_metrics,
                config.calibration_frames_per_second,
                node.calibration_samples_per_axis));
        }

        stages.production = std::move(production);
        
        if ((node.parameter_support_simplex || node.parameter_support_triangulated_2d) && !stages.production.HasExplicitParameterSupport()) {
            throw std::runtime_error("PrepareMotionSpaces: production space lost explicit parameter support");
        }

        const auto insertion = prepared.nodes.emplace(node.name, std::move(stages));
        if (!insertion.second) {
            throw std::runtime_error("PrepareMotionSpaces: duplicate node '" + node.name + "'");
        }
    }

    if (!reference_skeleton.has_value()) {
        throw std::runtime_error("PrepareMotionSpaces: spec contains no examples");
    }
    prepared.skeleton = *reference_skeleton;
    return prepared;
}

}  // namespace pmg
