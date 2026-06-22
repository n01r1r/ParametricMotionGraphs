#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace pmg {

struct CorpusAuditConfig {
    std::filesystem::path corpus_root;
    std::filesystem::path output_directory;
    std::size_t max_files = 0;  // 0 means unlimited.
    bool category_hints = true;
};

struct BvhFileAudit {
    std::string path;
    std::string parse_status;
    std::string failure_reason;
    std::string skeleton_signature;
    std::string channel_signature;
    int joint_count = 0;
    std::vector<std::string> joint_names;
    std::vector<std::string> root_channels;
    int frame_count = 0;
    double frame_time = 0.0;
    double duration_seconds = 0.0;
    double root_start[3]{};
    double root_end[3]{};
    double root_displacement[3]{};
    double root_path_length = 0.0;
    double mean_speed = 0.0;
    std::string category_hint_from_path;
    std::vector<std::string> candidate_labels;
    std::string recommended_use;
    std::vector<std::string> notes;
};

struct CorpusAuditResult {
    std::vector<BvhFileAudit> files;
};

CorpusAuditResult AuditBvhCorpus(const CorpusAuditConfig& config);
void WriteCorpusAudit(const CorpusAuditConfig& config, const CorpusAuditResult& result);

}  // namespace pmg
