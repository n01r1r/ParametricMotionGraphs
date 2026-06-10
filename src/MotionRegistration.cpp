#include "pmg/MotionRegistration.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace pmg {

std::vector<float> ContactAnchorPhases(
    const std::vector<ContactInterval>& intervals,
    int clip_frame_count) {
    std::vector<float> anchors;
    anchors.reserve(intervals.size() * 2);

    for (const ContactInterval& interval : intervals) {
        anchors.push_back(interval.StrikePhase(clip_frame_count));
        anchors.push_back(interval.LiftPhase(clip_frame_count));
    }

    std::sort(anchors.begin(), anchors.end());
    anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
    anchors.erase(
        std::remove_if(anchors.begin(), anchors.end(),
                       [](float phase) { return phase <= 0.0f || phase >= 1.0f; }),
        anchors.end());
    return anchors;
}

std::vector<TimeWarp> BuildRegistrationWarps(
    const std::vector<std::vector<float>>& example_anchor_phases) {
    if (example_anchor_phases.empty()) {
        throw std::runtime_error("BuildRegistrationWarps: no examples given");
    }

    const std::size_t anchor_count = example_anchor_phases.front().size();
    for (std::size_t example_index = 0; example_index < example_anchor_phases.size();
         ++example_index) {
        if (example_anchor_phases[example_index].size() != anchor_count) {
            throw std::runtime_error(
                "BuildRegistrationWarps: example " + std::to_string(example_index) +
                " has " + std::to_string(example_anchor_phases[example_index].size()) +
                " anchors, expected " + std::to_string(anchor_count) +
                "; examples must share the same contact structure");
        }
    }

    // Canonical domain: each anchor sits at the mean of the examples' phases.
    std::vector<float> canonical_anchors(anchor_count, 0.0f);
    for (const std::vector<float>& anchors : example_anchor_phases) {
        for (std::size_t anchor_index = 0; anchor_index < anchor_count; ++anchor_index) {
            canonical_anchors[anchor_index] += anchors[anchor_index];
        }
    }
    for (float& canonical : canonical_anchors) {
        canonical /= static_cast<float>(example_anchor_phases.size());
    }

    std::vector<TimeWarp> warps;
    warps.reserve(example_anchor_phases.size());
    for (const std::vector<float>& anchors : example_anchor_phases) {
        warps.push_back(TimeWarp::FromAnchors(canonical_anchors, anchors));
    }
    return warps;
}

void RegisterSpaceByContacts(
    ParametricMotionSpace& space,
    const Skeleton& skeleton,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings) {
    if (space.NumExamples() == 0) {
        throw std::runtime_error("RegisterSpaceByContacts: space has no examples");
    }

    std::vector<std::vector<float>> example_anchor_phases;
    example_anchor_phases.reserve(space.Examples().size());

    for (const ExampleMotion& example : space.Examples()) {
        const std::vector<ContactInterval> intervals =
            DetectContacts(skeleton, example.clip, contact_joints, settings);
        example_anchor_phases.push_back(
            ContactAnchorPhases(intervals, example.clip.NumFrames()));
    }

    space.SetExampleTimeWarps(BuildRegistrationWarps(example_anchor_phases));
}

}  // namespace pmg
