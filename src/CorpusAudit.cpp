#include "pmg/CorpusAudit.h"

#include "pmg/BvhLoader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace pmg {
namespace {

std::string ChannelName(BvhChannelType channel) {
    switch (channel) {
        case BvhChannelType::XPosition: return "Xposition";
        case BvhChannelType::YPosition: return "Yposition";
        case BvhChannelType::ZPosition: return "Zposition";
        case BvhChannelType::XRotation: return "Xrotation";
        case BvhChannelType::YRotation: return "Yrotation";
        case BvhChannelType::ZRotation: return "Zrotation";
    }
    return "unknown";
}

std::string Escape(std::string value) {
    std::string escaped;
    for (char character : value) {
        if (character == '\\' || character == '"') escaped += '\\';
        if (character == '\n') escaped += "\\n";
        else escaped += character;
    }
    return escaped;
}

std::string StableHash(const std::string& value) {
    std::uint64_t hash = 14695981039346656037ull;
    for (unsigned char byte : value) {
        hash ^= byte;
        hash *= 1099511628211ull;
    }
    std::ostringstream output;
    output << std::hex << std::setw(16) << std::setfill('0') << hash;
    return output.str();
}

std::string Lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::vector<std::string> LabelsFor(const std::string& path, double speed, double displacement) {
    const std::string lower = Lower(path);
    std::vector<std::string> labels;
    auto add = [&](const char* label, std::initializer_list<const char*> hints) {
        for (const char* hint : hints) if (lower.find(hint) != std::string::npos) { labels.emplace_back(label); return; }
    };
    add("walk_run", {"walk-run", "walk_run", "walktorun", "walktojog"});
    add("walk_turn", {"walks and turns", "walk_turn", "curve", "turn"});
    add("cartwheel", {"cartwheel", "acrobatic"});
    add("punch", {"punch", "boxing"});
    add("duck", {"duck", "roll"});
    add("step", {"step", "stair"});
    add("sit", {"sit"});
    add("jump", {"jump"});
    add("run", {"run", "jog"});
    add("walk", {"walk"});
    if (labels.empty()) labels.push_back("unknown");
    // Path hints are weak: locomotion labels require measurable root travel.
    if (displacement <= 1e-5 || speed <= 1e-5) {
        labels.erase(std::remove_if(labels.begin(), labels.end(), [](const std::string& label) {
            return label == "walk" || label == "run" || label == "walk_run" || label == "walk_turn";
        }), labels.end());
        if (labels.empty()) labels.push_back("unknown");
    }
    return labels;
}

void WriteStringArray(std::ostream& out, const std::vector<std::string>& values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << ',';
        out << '"' << Escape(values[i]) << '"';
    }
    out << ']';
}

std::map<std::string, std::vector<const BvhFileAudit*>> Groups(const CorpusAuditResult& result) {
    std::map<std::string, std::vector<const BvhFileAudit*>> groups;
    for (const auto& file : result.files) if (file.parse_status == "ok") groups[file.skeleton_signature].push_back(&file);
    return groups;
}

}  // namespace

CorpusAuditResult AuditBvhCorpus(const CorpusAuditConfig& config) {
    if (!std::filesystem::is_directory(config.corpus_root)) throw std::runtime_error("corpus-root is not a directory: " + config.corpus_root.string());
    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(config.corpus_root)) {
        if (entry.is_regular_file() && Lower(entry.path().extension().string()) == ".bvh") paths.push_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    if (config.max_files > 0 && paths.size() > config.max_files) paths.resize(config.max_files);

    CorpusAuditResult result;
    for (const auto& path : paths) {
        BvhFileAudit audit;
        audit.path = std::filesystem::relative(path, config.corpus_root).generic_string();
        try {
            const BvhData data = BvhLoader::Load(path.string());
            audit.parse_status = "ok";
            audit.joint_count = data.skeleton.NumJoints();
            std::ostringstream hierarchy;
            std::ostringstream channels;
            for (const Joint& joint : data.skeleton.joints) {
                audit.joint_names.push_back(joint.name);
                hierarchy << joint.parent_index << ':' << joint.name << ';';
                channels << joint.name << ':';
                for (BvhChannelType channel : joint.channels) channels << ChannelName(channel) << ',';
                channels << ';';
            }
            for (BvhChannelType channel : data.skeleton.joints.front().channels) audit.root_channels.push_back(ChannelName(channel));
            audit.channel_signature = StableHash(channels.str());
            audit.frame_count = data.clip.NumFrames();
            audit.frame_time = 1.0 / data.clip.frames_per_second;
            audit.duration_seconds = data.clip.DurationSeconds();
            const Vec3 start = data.clip.frames.front().root_position;
            const Vec3 end = data.clip.frames.back().root_position;
            const Vec3 delta = end - start;
            audit.root_start[0] = start.x; audit.root_start[1] = start.y; audit.root_start[2] = start.z;
            audit.root_end[0] = end.x; audit.root_end[1] = end.y; audit.root_end[2] = end.z;
            audit.root_displacement[0] = delta.x; audit.root_displacement[1] = delta.y; audit.root_displacement[2] = delta.z;
            for (std::size_t i = 1; i < data.clip.frames.size(); ++i) {
                const Vec3 step = data.clip.frames[i].root_position - data.clip.frames[i - 1].root_position;
                audit.root_path_length += std::sqrt(step.x * step.x + step.z * step.z);
            }
            const double horizontal_displacement = std::sqrt(delta.x * delta.x + delta.z * delta.z);
            audit.mean_speed = audit.duration_seconds > 0.0 ? audit.root_path_length / audit.duration_seconds : 0.0;
            audit.category_hint_from_path = config.category_hints ? Lower(std::filesystem::path(audit.path).parent_path().generic_string()) : "";
            audit.candidate_labels = LabelsFor(audit.path, audit.mean_speed, horizontal_displacement);
            const bool unknown = audit.candidate_labels.size() == 1 && audit.candidate_labels.front() == "unknown";
            audit.recommended_use = unknown ? "needs_manual_review" : "node_candidate";
            audit.notes.push_back("contact, cyclicity, axis, and heading metrics require manual review");
            std::ostringstream signature;
            signature << hierarchy.str() << '|' << channels.str() << "|dt=" << std::fixed << std::setprecision(5) << audit.frame_time;
            audit.skeleton_signature = StableHash(signature.str());
        } catch (const std::exception& error) {
            audit.parse_status = "failed";
            audit.failure_reason = error.what();
            audit.candidate_labels = {"unknown"};
            audit.recommended_use = "reject";
        }
        result.files.push_back(std::move(audit));
    }
    return result;
}

void WriteCorpusAudit(const CorpusAuditConfig& config, const CorpusAuditResult& result) {
    std::filesystem::create_directories(config.output_directory);
    std::ofstream json(config.output_directory / "manifest.json");
    std::ofstream csv(config.output_directory / "manifest.csv");
    if (!json || !csv) throw std::runtime_error("failed to open corpus audit manifest output");
    json << "{\n  \"files\": [\n";
    csv << "path,parse_status,failure_reason,skeleton_signature,joint_count,frame_count,frame_time,duration_seconds,root_path_length,mean_speed,candidate_labels,recommended_use\n";
    for (std::size_t i = 0; i < result.files.size(); ++i) {
        const auto& f = result.files[i];
        json << "    {\"path\":\"" << Escape(f.path) << "\",\"parse_status\":\"" << f.parse_status << "\",\"failure_reason\":\"" << Escape(f.failure_reason)
             << "\",\"skeleton_signature\":\"" << f.skeleton_signature << "\",\"joint_count\":" << f.joint_count << ",\"joint_names\":"; WriteStringArray(json, f.joint_names);
        json << ",\"channel_signature\":\"" << f.channel_signature << "\",\"root_channels\":"; WriteStringArray(json, f.root_channels);
        json << ",\"frame_count\":" << f.frame_count << ",\"frame_time\":" << f.frame_time << ",\"duration_seconds\":" << f.duration_seconds
             << ",\"root_start\":[" << f.root_start[0] << ',' << f.root_start[1] << ',' << f.root_start[2] << "],\"root_end\":[" << f.root_end[0] << ',' << f.root_end[1] << ',' << f.root_end[2]
             << "],\"root_displacement\":[" << f.root_displacement[0] << ',' << f.root_displacement[1] << ',' << f.root_displacement[2] << "],\"root_path_length\":" << f.root_path_length << ",\"mean_speed\":" << f.mean_speed
             << ",\"estimated_up_axis\":\"unknown\",\"estimated_forward_axis\":\"unknown\",\"heading_change_degrees\":null,\"floor_height_estimate\":null,\"foot_joint_candidates\":[],\"left_contact_ratio\":null,\"right_contact_ratio\":null,\"contact_quality\":\"unknown\",\"cyclicity_score\":null,\"locomotion_score\":null,\"turning_score\":null,\"category_hint_from_path\":\"" << Escape(f.category_hint_from_path) << "\",\"candidate_labels\":"; WriteStringArray(json, f.candidate_labels);
        json << ",\"recommended_use\":\"" << f.recommended_use << "\",\"notes\":"; WriteStringArray(json, f.notes); json << '}' << (i + 1 == result.files.size() ? "\n" : ",\n");
        std::ostringstream labels; for (std::size_t j = 0; j < f.candidate_labels.size(); ++j) { if (j) labels << '|'; labels << f.candidate_labels[j]; }
        csv << '"' << Escape(f.path) << "\",\"" << f.parse_status << "\",\"" << Escape(f.failure_reason) << "\",\"" << f.skeleton_signature << "\"," << f.joint_count << ',' << f.frame_count << ',' << f.frame_time << ',' << f.duration_seconds << ',' << f.root_path_length << ',' << f.mean_speed << ",\"" << labels.str() << "\",\"" << f.recommended_use << "\"\n";
    }
    json << "  ]\n}\n";

    const auto groups = Groups(result);
    std::ofstream skeletons(config.output_directory / "skeleton_groups.json");
    skeletons << "{\n  \"groups\": [\n";
    std::size_t group_index = 0;
    for (const auto& [signature, files] : groups) {
        skeletons << "    {\"group_id\":\"skeleton_group_" << std::setw(4) << std::setfill('0') << ++group_index << "\",\"skeleton_signature\":\"" << signature << "\",\"file_count\":" << files.size() << ",\"joint_count\":" << files.front()->joint_count << ",\"representative_file\":\"" << Escape(files.front()->path) << "\",\"recommended_priority\":\"" << (files.size() >= 2 ? "high" : "low") << "\"}" << (group_index == groups.size() ? "\n" : ",\n");
    }
    skeletons << "  ]\n}\n";

    std::ofstream candidates(config.output_directory / "candidate_groups.json");
    candidates << "{\n  \"candidate_groups\": [\n";
    const char* names[] = {"run_1d_candidate","run_2d_candidate","walk_run_transition_candidate","walk_turn_candidate","cartwheel_candidate","punch_candidate","duck_candidate","step_sit_jump_candidate"};
    for (std::size_t i = 0; i < std::size(names); ++i) candidates << "    {\"name\":\"" << names[i] << "\",\"source_files\":[],\"skeleton_group_id\":\"\",\"candidate_parameter\":\"unknown\",\"confidence\":\"low\",\"blocking_issues\":[\"manual evidence review required\"],\"next_action\":\"manual_review\"}" << (i + 1 == std::size(names) ? "\n" : ",\n");
    candidates << "  ]\n}\n";

    std::ofstream report(config.output_directory / "gap_closure_report.md");
    report << "# CMU BVH Gap-Closure Audit\n\nThis report is heuristic evidence, not reproduction of paper Section 5.\n\n## PASS candidates\n\nNone automatically promoted.\n\n## SOFT PASS candidates\n\nParsed, labeled files require manual window/contact review.\n\n## FAIL candidates\n\n";
    for (const auto& f : result.files) if (f.parse_status == "failed") report << "- `" << f.path << "`: " << f.failure_reason << "\n";
    report << "\n## UNKNOWN candidates\n\nUnlabeled or unsupported metrics remain unknown.\n\n## Evidence summary\n\n- Best skeleton group: " << (groups.empty() ? "none" : groups.begin()->first) << "\n- Best walk/run/transition/cartwheel/punch/duck candidates: manual review required\n- Current blockers: contact quality, cyclicity, heading, axis convention, compatible walk_2d skeleton comparison\n\n## Recommended next PR\n\nAdd manual candidate-window annotation support. Multi-node PMG construction remains gated.\n";
}

}  // namespace pmg
