#ifdef _WIN32
#define _CALL_REPORTDLG _CALL_REPORTFAULT
#endif
#include <initializer_list>
#include "pmg_test.h"
#include "pmg/CorpusAudit.h"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#define REQUIRE(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)

void WriteBvh(const std::filesystem::path& path, const char* joint_name, int end_x = 1) {
    std::ofstream out(path);
    out << "HIERARCHY\nROOT Hips\n{\nOFFSET 0 0 0\nCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\nJOINT " << joint_name
        << "\n{\nOFFSET 0 -1 0\nCHANNELS 3 Zrotation Xrotation Yrotation\nEnd Site\n{ OFFSET 0 -1 0 }\n}\n}\nMOTION\nFrames: 2\nFrame Time: 0.0333333\n0 0 0 0 0 0 0 0 0\n" << end_x << " 0 0 0 0 0 0 0 0\n";
}

int RunTest() {
    const auto root = std::filesystem::temp_directory_path() / ("pmg_corpus_audit_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root / "Walk");
    WriteBvh(root / "Walk/a.bvh", "Foot");
    WriteBvh(root / "Walk/b.bvh", "Foot");
    WriteBvh(root / "Walk/different.bvh", "Hand");
    std::filesystem::create_directories(root / "Run");
    WriteBvh(root / "Run/stationary_run.bvh", "Foot", 0);
    WriteBvh(root / "Run/moving_run.bvh", "Foot");
    std::ofstream(root / "broken.bvh") << "not BVH";
    pmg::CorpusAuditConfig config{root, root / "out"};
    const auto result = pmg::AuditBvhCorpus(config);
    REQUIRE(result.files.size() == 6);
    const pmg::BvhFileAudit* a = nullptr;
    const pmg::BvhFileAudit* b = nullptr;
    const pmg::BvhFileAudit* different = nullptr;
    int failures = 0;
    const pmg::BvhFileAudit* stationary = nullptr;
    for (const auto& file : result.files) {
        if (file.parse_status == "failed") ++failures;
        else if (file.path.ends_with("stationary_run.bvh")) stationary = &file;
        else if (file.path.ends_with("a.bvh")) a = &file;
        else if (file.path.ends_with("b.bvh")) b = &file;
        else different = &file;
    }
    REQUIRE(failures == 1 && a && b && different && stationary);
    REQUIRE(a->candidate_labels == std::vector<std::string>{"walk"});
    REQUIRE(stationary->recommended_use != "node_candidate");
    REQUIRE(a->skeleton_signature == b->skeleton_signature);
    REQUIRE(a->skeleton_signature != different->skeleton_signature);
    pmg::WriteCorpusAudit(config, result);
    for (const char* name : {"manifest.json", "manifest.csv", "skeleton_groups.json", "candidate_groups.json", "gap_closure_report.md"}) REQUIRE(std::filesystem::exists(config.output_directory / name));
    std::ifstream manifest(config.output_directory / "manifest.json");
    const std::string text((std::istreambuf_iterator<char>(manifest)), {});
    REQUIRE(text.find("\"cyclicity_score\":null") != std::string::npos);
    REQUIRE(text.find("\"horizontal_displacement\":") != std::string::npos);
    std::ifstream skeletons(config.output_directory / "skeleton_groups.json");
    const std::string skeleton_text((std::istreambuf_iterator<char>(skeletons)), {});
    REQUIRE(skeleton_text.find("\"candidate_label_counts\":") != std::string::npos);
    REQUIRE(skeleton_text.find("\"frame_time_values\":") != std::string::npos);
    skeletons.close();
    std::ifstream candidates(config.output_directory / "candidate_groups.json");
    const std::string candidate_text((std::istreambuf_iterator<char>(candidates)), {});
    REQUIRE(candidate_text.find("\"run_1d_candidate\"") != std::string::npos);
    REQUIRE(candidate_text.find("\"source_files\":[\"Run/moving_run.bvh\"") != std::string::npos);
    candidates.close();
    pmg::CorpusAuditConfig limited{root, root / "limited"};
    limited.max_files = 2;
    REQUIRE(pmg::AuditBvhCorpus(limited).files.size() == 2);
    manifest.close();
    std::filesystem::remove_all(root);
    return 0;
}

int main() {
    try { return RunTest(); }
    catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
}
