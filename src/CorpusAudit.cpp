#include "pmg/CorpusAudit.h"

#include "pmg/BvhLoader.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
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

bool HasLabel(const BvhFileAudit& file, const std::string& label) {
    return std::find(file.candidate_labels.begin(), file.candidate_labels.end(), label) != file.candidate_labels.end();
}

std::string GroupId(std::size_t index) {
    std::ostringstream out;
    out << "skeleton_group_" << std::setw(4) << std::setfill('0') << index + 1;
    return out.str();
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
            double absolute_turn = 0.0;
            std::size_t turn_count = 0;
            Vec3 first_direction{};
            Vec3 last_direction{};
            bool have_direction = false;
            for (std::size_t i = 1; i < data.clip.frames.size(); ++i) {
                const Vec3 step = data.clip.frames[i].root_position - data.clip.frames[i - 1].root_position;
                const double length = std::sqrt(step.x * step.x + step.z * step.z);
                audit.root_path_length += length;
                if (length <= 1e-8) continue;
                const Vec3 direction{static_cast<float>(step.x / length), 0.0f, static_cast<float>(step.z / length)};
                if (!have_direction) first_direction = direction;
                else absolute_turn += std::abs(std::atan2(last_direction.x * direction.z - last_direction.z * direction.x, last_direction.x * direction.x + last_direction.z * direction.z));
                last_direction = direction;
                have_direction = true;
                ++turn_count;
            }
            audit.horizontal_displacement = std::sqrt(delta.x * delta.x + delta.z * delta.z);
            audit.mean_speed = audit.duration_seconds > 0.0 ? audit.root_path_length / audit.duration_seconds : 0.0;
            audit.heading_change_degrees = turn_count > 1 ? std::atan2(first_direction.x * last_direction.z - first_direction.z * last_direction.x, first_direction.x * last_direction.x + first_direction.z * last_direction.z) * 180.0 / 3.14159265358979323846 : 0.0;
            audit.turning_score = turn_count > 1 ? std::min(1.0, absolute_turn / ((turn_count - 1) * 0.5)) : 0.0;
            audit.locomotion_score = audit.root_path_length > 1e-8 ? std::clamp(audit.horizontal_displacement / audit.root_path_length, 0.0, 1.0) : 0.0;
            audit.category_hint_from_path = config.category_hints ? Lower(std::filesystem::path(audit.path).parent_path().generic_string()) : "";
            audit.candidate_labels = LabelsFor(audit.path, audit.mean_speed, audit.horizontal_displacement);
            const bool unknown = audit.candidate_labels.size() == 1 && audit.candidate_labels.front() == "unknown";
            const bool conservative_locomotion_pass = !unknown && audit.locomotion_score >= 0.8 && audit.horizontal_displacement > 1e-5 && (HasLabel(audit, "walk") || HasLabel(audit, "run"));
            audit.recommended_use = conservative_locomotion_pass ? "node_candidate" : (unknown ? "needs_manual_review" : "soft_pass");
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
             << "],\"root_displacement\":[" << f.root_displacement[0] << ',' << f.root_displacement[1] << ',' << f.root_displacement[2] << "],\"horizontal_displacement\":" << f.horizontal_displacement << ",\"root_path_length\":" << f.root_path_length << ",\"mean_speed\":" << f.mean_speed
             << ",\"estimated_up_axis\":\"unknown\",\"estimated_forward_axis\":\"unknown\",\"heading_change_degrees\":" << f.heading_change_degrees << ",\"floor_height_estimate\":null,\"foot_joint_candidates\":[],\"left_contact_ratio\":null,\"right_contact_ratio\":null,\"contact_quality\":\"unknown\",\"cyclicity_score\":null,\"locomotion_score\":" << f.locomotion_score << ",\"turning_score\":" << f.turning_score << ",\"category_hint_from_path\":\"" << Escape(f.category_hint_from_path) << "\",\"candidate_labels\":"; WriteStringArray(json, f.candidate_labels);
        json << ",\"recommended_use\":\"" << f.recommended_use << "\",\"notes\":"; WriteStringArray(json, f.notes); json << '}' << (i + 1 == result.files.size() ? "\n" : ",\n");
        std::ostringstream labels; for (std::size_t j = 0; j < f.candidate_labels.size(); ++j) { if (j) labels << '|'; labels << f.candidate_labels[j]; }
        csv << '"' << Escape(f.path) << "\",\"" << f.parse_status << "\",\"" << Escape(f.failure_reason) << "\",\"" << f.skeleton_signature << "\"," << f.joint_count << ',' << f.frame_count << ',' << f.frame_time << ',' << f.duration_seconds << ',' << f.root_path_length << ',' << f.mean_speed << ",\"" << labels.str() << "\",\"" << f.recommended_use << "\"\n";
    }
    json << "  ]\n}\n";

    const auto groups = Groups(result);
    std::ofstream skeletons(config.output_directory / "skeleton_groups.json");
    skeletons << "{\n  \"groups\": [\n";
    std::size_t group_index = 0;
    std::map<std::string, std::string> group_ids;
    const BvhFileAudit* largest_group_file = nullptr;
    const BvhFileAudit* best_locomotion_file = nullptr;
    std::size_t largest_group_size = 0;
    for (const auto& [signature, files] : groups) {
        const std::string id = GroupId(group_index++); group_ids[signature] = id;
        std::map<std::string, int> counts; std::set<double> frame_times;
        for (const auto* file : files) { frame_times.insert(file->frame_time); for (const auto& label : file->candidate_labels) ++counts[label]; if (!best_locomotion_file || file->locomotion_score > best_locomotion_file->locomotion_score) best_locomotion_file = file; }
        if (files.size() > largest_group_size) { largest_group_size = files.size(); largest_group_file = files.front(); }
        const int useful = counts["walk"] + counts["run"] + counts["walk_run"] + counts["walk_turn"];
        const char* priority = useful >= 2 ? "high" : (useful == 1 ? "medium" : "low");
        skeletons << "    {\"group_id\":\"" << id << "\",\"skeleton_signature\":\"" << signature << "\",\"file_count\":" << files.size() << ",\"frame_time_values\":[";
        std::size_t n = 0; for (double value : frame_times) skeletons << (n++ ? "," : "") << value;
        skeletons << "],\"joint_count\":" << files.front()->joint_count << ",\"representative_file\":\"" << Escape(files.front()->path) << "\",\"candidate_label_counts\":{";
        n = 0; for (const auto& [label, count] : counts) skeletons << (n++ ? "," : "") << "\"" << label << "\":" << count;
        skeletons << "},\"recommended_priority\":\"" << priority << "\",\"reasons\":[\"" << useful << " useful locomotion labels\",\"" << files.size() << " compatible files\"]}" << (group_index == groups.size() ? "\n" : ",\n");
    }
    skeletons << "  ]\n}\n";

    std::ofstream candidates(config.output_directory / "candidate_groups.json");
    candidates << "{\n  \"candidate_groups\": [\n";
    struct CandidateSpec { const char* name; std::initializer_list<const char*> labels; bool turning; };
    const CandidateSpec specs[] = {{"run_1d_candidate",{"run"},false},{"run_2d_candidate",{"run"},true},{"walk_run_transition_candidate",{"walk_run"},false},{"walk_turn_candidate",{"walk_turn"},true},{"cartwheel_candidate",{"cartwheel"},false},{"punch_candidate",{"punch"},false},{"duck_candidate",{"duck"},false},{"step_sit_jump_candidate",{"step","sit","jump"},false}};
    for (std::size_t i = 0; i < std::size(specs); ++i) {
        std::vector<const BvhFileAudit*> matches;
        for (const auto& file : result.files) if (file.parse_status == "ok" && (!specs[i].turning || file.turning_score > 0.1)) for (const char* label : specs[i].labels) if (HasLabel(file, label)) { matches.push_back(&file); break; }
        candidates << "    {\"name\":\"" << specs[i].name << "\",\"source_files\":["; for (std::size_t j = 0; j < matches.size(); ++j) candidates << (j ? "," : "") << "\"" << Escape(matches[j]->path) << "\"";
        const std::string id = matches.empty() ? "" : group_ids[matches.front()->skeleton_signature];
        const bool hard = !matches.empty() && std::all_of(matches.begin(), matches.end(), [](const auto* file){ return file->recommended_use == "node_candidate"; });
        candidates << "],\"skeleton_group_id\":\"" << id << "\",\"candidate_parameter\":\"unknown\",\"confidence\":\"" << (hard ? "medium" : "low") << "\",\"blocking_issues\":[\"contact, cyclicity, axis, and window evidence unknown\"],\"next_action\":\"" << (matches.empty() ? "find_source_files" : "manually_review_and_extract_windows") << "\"}" << (i + 1 == std::size(specs) ? "\n" : ",\n");
    }
    candidates << "  ]\n}\n";

    std::ofstream report(config.output_directory / "gap_closure_report.md");
    report << "# CMU BVH Gap-Closure Audit\n\nThis report is heuristic evidence, not reproduction of paper Section 5.\n\n## FAIL candidates\n\n";
    for (const auto& f : result.files) if (f.parse_status == "failed") report << "- `" << f.path << "`: " << f.failure_reason << "\n";
    auto best = [&](const char* label) { const BvhFileAudit* found = nullptr; for (const auto& f : result.files) if (HasLabel(f, label) && (!found || f.locomotion_score > found->locomotion_score)) found = &f; return found ? found->path : "none"; };
    report << "\n## Evidence summary\n\n- Largest skeleton group: " << (largest_group_file ? group_ids[largest_group_file->skeleton_signature] : "none") << " (" << largest_group_size << " files)\n- Best locomotion skeleton group: " << (best_locomotion_file ? group_ids[best_locomotion_file->skeleton_signature] : "none") << "\n- Best walk candidates: " << best("walk") << "\n- Best run candidates: " << best("run") << "\n- Best walk-run transition candidates: " << best("walk_run") << "\n- Best cartwheel candidates: " << best("cartwheel") << "\n- Best punch/duck candidates: " << best("punch") << "; " << best("duck") << "\n- Blockers: contact quality, cyclicity, axis convention, candidate windows, compatible-skeleton comparison\n\n## Recommended next PR\n\nManually review ranked files and extract candidate windows; do not build a PMG node until blockers close.\n";
}

}  // namespace pmg
