#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"
#include "pmg/SkeletonCompatibility.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>


namespace {

std::string LowercaseCopy(std::string text) {
    std::transform(
        text.begin(), text.end(), text.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return text;
}

const char* ChannelName(pmg::BvhChannelType channel) {
    switch (channel) {
        case pmg::BvhChannelType::XPosition: return "Xposition";
        case pmg::BvhChannelType::YPosition: return "Yposition";
        case pmg::BvhChannelType::ZPosition: return "Zposition";
        case pmg::BvhChannelType::XRotation: return "Xrotation";
        case pmg::BvhChannelType::YRotation: return "Yrotation";
        case pmg::BvhChannelType::ZRotation: return "Zrotation";
    }
    return "Unknown";
}

std::optional<int> FindJointExact(
    const pmg::Skeleton& skeleton,
    const std::string& joint_name) {
    const std::string target = LowercaseCopy(joint_name);

    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        if (LowercaseCopy(skeleton.joints[joint_index].name) == target) {
            return joint_index;
        }
    }

    return std::nullopt;
}

float HorizontalLength(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

int PrintBvhSummary(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);
    std::cout << "BVH: " << path << "\n";
    std::cout << "joints: " << data.skeleton.NumJoints() << "\n";
    std::cout << "frames: " << data.clip.NumFrames() << "\n";
    std::cout << "fps: " << data.clip.frames_per_second << "\n";
    return 0;
}

int ListBvhJoints(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);

    for (int joint_index = 0; joint_index < data.skeleton.NumJoints(); ++joint_index) {
        const pmg::Joint& joint = data.skeleton.joints[joint_index];
        std::cout << joint_index
                  << " name=" << joint.name
                  << " parent=" << joint.parent_index
                  << " offset=("
                  << joint.offset.x << ", "
                  << joint.offset.y << ", "
                  << joint.offset.z << ")"
                  << " channels=";

        for (int channel_index = 0;
             channel_index < static_cast<int>(joint.channels.size());
             ++channel_index) {
            if (channel_index > 0) {
                std::cout << ",";
            }
            std::cout << ChannelName(joint.channels[channel_index]);
        }

        std::cout << "\n";
    }

    return 0;
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

std::vector<int> ResolveJointList(
    const pmg::Skeleton& skeleton,
    const std::string& comma_names) {
    std::vector<int> indices;
    for (const std::string& name : SplitCommaList(comma_names)) {
        const std::optional<int> index = FindJointExact(skeleton, name);
        if (!index) {
            throw std::runtime_error("unknown joint '" + name + "'");
        }
        indices.push_back(*index);
    }
    if (indices.empty()) {
        throw std::runtime_error("no contact joints given");
    }
    return indices;
}

// Print detected contact intervals and the anchor phases registration would
// use. Diagnostic for picking contact joints and verifying that clips meant
// to share a motion space expose the same contact structure.
int InspectContacts(const std::string& path, const std::string& joints_csv) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);
    const std::vector<int> joints = ResolveJointList(data.skeleton, joints_csv);

    const pmg::ContactDetectionSettings settings =
        pmg::EstimateContactSettings(data.skeleton, data.clip, joints);
    const std::vector<pmg::ContactInterval> intervals =
        pmg::DetectContacts(data.skeleton, data.clip, joints, settings);

    std::cout << "bvh=" << path << "\n";
    std::cout << "frames=" << data.clip.NumFrames()
              << " fps=" << data.clip.frames_per_second << "\n";
    std::cout << "height_threshold=" << settings.height_threshold
              << " speed_threshold=" << settings.speed_threshold << "\n";
    std::cout << "contacts=" << intervals.size() << "\n";
    for (const pmg::ContactInterval& interval : intervals) {
        std::cout << "  joint=" << data.skeleton.joints[interval.joint_index].name
                  << " frames=[" << interval.first_frame << ", " << interval.last_frame << "]"
                  << " strike_phase=" << interval.StrikePhase(data.clip.NumFrames())
                  << " lift_phase=" << interval.LiftPhase(data.clip.NumFrames())
                  << "\n";
    }

    const std::vector<float> anchors =
        pmg::ContactAnchorPhases(intervals, data.clip.NumFrames());
    std::cout << "anchor_count=" << anchors.size() << "\n";
    std::cout << "anchors=";
    for (std::size_t i = 0; i < anchors.size(); ++i) {
        if (i > 0) {
            std::cout << ",";
        }
        std::cout << anchors[i];
    }
    std::cout << "\n";
    return 0;
}

}  // namespace

namespace pmgcli {

std::optional<int> TryRunBvhCommand(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "--bvh" && argc == 3) {
        return PrintBvhSummary(argv[2]);
    }
    if (command == "--list-bvh-joints" && argc == 3) {
        return ListBvhJoints(argv[2]);
    }
    if (command == "--inspect-contacts" && argc == 4) {
        return InspectContacts(argv[2], argv[3]);
    }
    return std::nullopt;
}

}  // namespace pmgcli
