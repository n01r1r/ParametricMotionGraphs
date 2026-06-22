#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/CandidateWindowExtractor.h"
#include "pmg/CorpusAudit.h"
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

int AuditCorpus(int argc, char** argv) {
    pmg::CorpusAuditConfig config;
    for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--category-hints" || option == "--candidate-windows" || option == "--verbose") continue;
        if (index + 1 >= argc) throw std::runtime_error("missing value for option: " + option);
        const std::string value = argv[++index];
        if (option == "--corpus-root") config.corpus_root = value;
        else if (option == "--out") config.output_directory = value;
        else if (option == "--max-files") {
            config.max_files = std::stoull(value);
            if (config.max_files == 0) throw std::runtime_error("max-files must be positive");
        }
        else if (option == "--include" && value != "**/*.bvh") throw std::runtime_error("audit-corpus supports only --include **/*.bvh");
        else throw std::runtime_error("unknown audit-corpus option: " + option);
    }
    if (config.corpus_root.empty() || config.output_directory.empty()) throw std::runtime_error("audit-corpus requires --corpus-root and --out");
    const pmg::CorpusAuditResult result = pmg::AuditBvhCorpus(config);
    pmg::WriteCorpusAudit(config, result);
    std::cout << "audited " << result.files.size() << " BVH files into " << config.output_directory << "\n";
    return 0;
}

int ParsePositiveInt(const std::string& text, const char* name) {
    std::size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (consumed != text.size() || value < 1) {
        throw std::runtime_error(std::string(name) + " must be a positive integer");
    }
    return value;
}

int ExtractCandidateWindows(int argc, char** argv) {
    if (argc < 4) {
        throw std::runtime_error("usage: pmg_cli --extract-candidate-windows file.bvh --min-frames N --max-frames N --stride N --top-k K --output-md path --output-csv path [--output-candidates path]");
    }
    const std::string source_path = argv[2];
    pmg::CandidateWindowExtractionConfig config;
    std::filesystem::path markdown_path;
    std::filesystem::path csv_path;
    std::filesystem::path candidates_path;
    for (int index = 3; index < argc; index += 2) {
        if (index + 1 >= argc) throw std::runtime_error("missing value for option: " + std::string(argv[index]));
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (option == "--min-frames") config.min_length_frames = ParsePositiveInt(value, "min-frames");
        else if (option == "--max-frames") config.max_length_frames = ParsePositiveInt(value, "max-frames");
        else if (option == "--stride") config.stride_frames = ParsePositiveInt(value, "stride");
        else if (option == "--top-k") config.top_k = ParsePositiveInt(value, "top-k");
        else if (option == "--output-md") markdown_path = value;
        else if (option == "--output-csv") csv_path = value;
        else if (option == "--output-candidates") candidates_path = value;
        else throw std::runtime_error("unknown candidate extraction option: " + option);
    }
    if (markdown_path.empty() || csv_path.empty()) {
        throw std::runtime_error("candidate extraction requires --output-md and --output-csv");
    }

    const pmg::BvhData data = pmg::BvhLoader::Load(source_path);
    const auto candidates = pmg::ExtractCandidateMotionWindows(data.skeleton, data.clip, config);
    for (const auto& path : {markdown_path, csv_path, candidates_path}) {
        if (path.empty()) continue;
        if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream csv(csv_path);
    std::ofstream markdown(markdown_path);
    if (!csv || !markdown) throw std::runtime_error("failed to open candidate extraction output");
    if (!candidates_path.empty()) {
        std::ofstream candidate_output(candidates_path);
        if (!candidate_output) throw std::runtime_error("failed to open candidate JSON output");
        pmg::WriteCandidateWindowsJson(candidate_output, source_path, config, candidates);
    }
    csv << "source,start_frame,end_frame,duration_seconds,score,root_displacement,heading_delta_radians,reason\n";
    markdown << "# Candidate motion windows\n\n"
             << "- Source BVH: `" << source_path << "`\n"
             << "- Native frames: " << data.clip.NumFrames() << "\n"
             << "- FPS: " << data.clip.frames_per_second << "\n\n";
    if (candidates.empty()) markdown << "> Warning: no plausible candidate found.\n";
    else markdown << "| Start | End | Duration (s) | Score | Root displacement | Heading delta (rad) | Reason |\n|---:|---:|---:|---:|---:|---:|---|\n";
    for (const auto& candidate : candidates) {
        const float duration = static_cast<float>(candidate.end_frame - candidate.start_frame + 1) / data.clip.frames_per_second;
        csv << '"' << source_path << "\"," << candidate.start_frame << ',' << candidate.end_frame << ',' << duration << ',' << candidate.score << ',' << candidate.root_displacement << ',' << candidate.heading_delta << ",\"" << candidate.reason << "\"\n";
        markdown << "| " << candidate.start_frame << " | " << candidate.end_frame << " | " << duration << " | " << candidate.score << " | " << candidate.root_displacement << " | " << candidate.heading_delta << " | " << candidate.reason << " |\n";
    }
    std::cout << "wrote " << candidates.size() << " candidate windows\n";
    return 0;
}

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
    while (std::getline(stream, item)) {
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
    std::istringstream stream(comma_names);
    std::string name;
    while (std::getline(stream, name, ',')) {
        if (name.empty()) {
            continue;
        }
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

std::string TrimLeftCopy(const std::string& text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    return text.substr(first);
}

bool StartsWithAfterTrim(const std::string& line, const std::string& prefix) {
    const std::string trimmed = TrimLeftCopy(line);
    return trimmed.rfind(prefix, 0) == 0;
}

std::vector<std::string> ReadTextLines(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("failed to open BVH for reading: " + path.string());
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::size_t FindFirstLineStartingWith(
    const std::vector<std::string>& lines,
    const std::string& prefix,
    std::size_t start_index) {
    for (std::size_t line_index = start_index; line_index < lines.size(); ++line_index) {
        if (StartsWithAfterTrim(lines[line_index], prefix)) {
            return line_index;
        }
    }

    throw std::runtime_error("BVH recut: missing line starting with '" + prefix + "'");
}

std::size_t FindFrameTimeLine(
    const std::vector<std::string>& lines,
    std::size_t start_index) {
    for (std::size_t line_index = start_index; line_index < lines.size(); ++line_index) {
        const std::string trimmed = TrimLeftCopy(lines[line_index]);
        if (trimmed.rfind("Frame Time:", 0) == 0) {
            return line_index;
        }
        if (trimmed.rfind("Frame", 0) == 0 &&
            trimmed.find("Time:") != std::string::npos) {
            return line_index;
        }
    }

    throw std::runtime_error("BVH recut: missing Frame Time line");
}

int ParseNonNegativeInt(const std::string& text, const char* label) {
    std::size_t parsed_count = 0;
    const int value = std::stoi(text, &parsed_count);
    if (parsed_count != text.size()) {
        throw std::runtime_error(std::string(label) + " must be an integer: " + text);
    }
    if (value < 0) {
        throw std::runtime_error(std::string(label) + " must be non-negative");
    }
    return value;
}

// Raw BVH recut: copy the original hierarchy and channel rows verbatim, then
// replace only the Frames count and keep the requested inclusive frame range.
// This deliberately does not call ExtractFirstCycle or contact detection.
void WriteRawBvhRecut(
    const std::filesystem::path& input_path,
    const std::filesystem::path& output_path,
    int first_frame,
    int last_frame) {
    if (last_frame < first_frame) {
        throw std::runtime_error("BVH recut: last_frame must be >= first_frame");
    }

    const std::vector<std::string> lines = ReadTextLines(input_path);
    const std::size_t frames_line = FindFirstLineStartingWith(lines, "Frames:", 0);
    const std::size_t frame_time_line = FindFrameTimeLine(lines, frames_line + 1);
    const std::size_t motion_begin = frame_time_line + 1;

    if (motion_begin >= lines.size()) {
        throw std::runtime_error("BVH recut: no motion frame rows after Frame Time line");
    }

    const int available_frames = static_cast<int>(lines.size() - motion_begin);
    if (first_frame >= available_frames || last_frame >= available_frames) {
        std::ostringstream message;
        message << "BVH recut: requested frame range ["
                << first_frame << ", " << last_frame
                << "] outside available frame rows [0, "
                << (available_frames - 1) << "]";
        throw std::runtime_error(message.str());
    }

    const int output_frame_count = last_frame - first_frame + 1;

    const std::filesystem::path parent = output_path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }

    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("failed to open BVH for writing: " + output_path.string());
    }

    for (std::size_t line_index = 0; line_index < frames_line; ++line_index) {
        output << lines[line_index] << "\n";
    }

    output << "Frames: " << output_frame_count << "\n";

    for (std::size_t line_index = frames_line + 1;
         line_index <= frame_time_line;
         ++line_index) {
        output << lines[line_index] << "\n";
    }

    for (int frame_index = first_frame; frame_index <= last_frame; ++frame_index) {
        output << lines[motion_begin + static_cast<std::size_t>(frame_index)] << "\n";
    }

    std::cout << "wrote recut BVH: " << output_path.string()
              << " frames=[" << first_frame << ", " << last_frame << "]"
              << " count=" << output_frame_count << "\n";
}

int ExportBvhRecut(int argc, char** argv) {
    if (argc != 6) {
        throw std::runtime_error(
            "usage: pmg_cli --export-bvh-recut input.bvh output.bvh first_frame last_frame");
    }

    const std::filesystem::path input_path = argv[2];
    const std::filesystem::path output_path = argv[3];
    const int first_frame = ParseNonNegativeInt(argv[4], "first_frame");
    const int last_frame = ParseNonNegativeInt(argv[5], "last_frame");

    WriteRawBvhRecut(input_path, output_path, first_frame, last_frame);
    return 0;
}

struct KnownCyclicRecut {
    const char* input_filename;
    const char* output_filename;
    int first_frame;
    int last_frame;
};

int ExportKnownCyclicRecuts(int argc, char** argv) {
    if (argc != 6) {
        throw std::runtime_error(
            "usage: pmg_cli --export-known-cyclic-recuts --bvh-dir BVH --output-dir BVH/recut");
    }

    std::optional<std::filesystem::path> bvh_dir;
    std::optional<std::filesystem::path> output_dir;

    for (int arg_index = 2; arg_index < argc; arg_index += 2) {
        const std::string key = argv[arg_index];
        if (arg_index + 1 >= argc) {
            throw std::runtime_error("missing value for option: " + key);
        }

        if (key == "--bvh-dir") {
            bvh_dir = std::filesystem::path(argv[arg_index + 1]);
            continue;
        }
        if (key == "--output-dir") {
            output_dir = std::filesystem::path(argv[arg_index + 1]);
            continue;
        }

        throw std::runtime_error("unknown --export-known-cyclic-recuts option: " + key);
    }

    if (!bvh_dir || !output_dir) {
        throw std::runtime_error(
            "--export-known-cyclic-recuts requires --bvh-dir and --output-dir");
    }

    constexpr KnownCyclicRecut kRecuts[] = {
        {"walkCurve.bvh", "walkCurve_recut_086_119.bvh", 86, 119},
        {"walkMoreCurve.bvh", "walkMoreCurve_recut_076_110.bvh", 76, 110},
        {"walkTightCurve.bvh", "walkTightCurve_recut_058_095.bvh", 58, 95},
        {"jogCurve.bvh", "jogCurve_recut_039_063.bvh", 39, 63},
    };

    std::filesystem::create_directories(*output_dir);

    for (const KnownCyclicRecut& recut : kRecuts) {
        WriteRawBvhRecut(
            *bvh_dir / recut.input_filename,
            *output_dir / recut.output_filename,
            recut.first_frame,
            recut.last_frame);
    }

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
    if (command == "--extract-candidate-windows") {
        return ExtractCandidateWindows(argc, argv);
    }
    if (command == "audit-corpus" || command == "--audit-corpus") return AuditCorpus(argc, argv);
    if (command == "--inspect-contacts" && argc == 4) {
        return InspectContacts(argv[2], argv[3]);
    }
    if (command == "--export-bvh-recut") {
        return ExportBvhRecut(argc, argv);
    }
    if (command == "--export-known-cyclic-recuts") {
        return ExportKnownCyclicRecuts(argc, argv);
    }
    return std::nullopt;
}

}  // namespace pmgcli
