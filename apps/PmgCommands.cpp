#include "PmgCommandModules.h"

#include <exception>
#include <iostream>
#include <optional>

namespace {

void PrintUsage() {
    std::cerr << "Usage:\n"
              << "  pmg_cli --bvh path/to/file.bvh\n"
              << "  pmg_cli --list-bvh-joints path/to/file.bvh\n"
              << "  pmg_cli --inspect-transition source.bvh target.bvh\n"
              << "  pmg_cli --compare-transition-conventions source.bvh target.bvh\n"
              << "  pmg_cli --dump-distance-grid source.bvh target.bvh out.csv\n"
              << "  pmg_cli --calibrate-thresholds bvh_directory [manifest.txt]\n"
              << "  pmg_cli --validate-graph-spec graph_spec.txt\n"
              << "  pmg_cli --diagnose-graph-edge graph_spec.txt source_node target_node [--tgood X --tbad Y]\n"
              << "  pmg_cli --build-graph graph_spec.txt out.pmg [--tgood X --tbad Y --blend-frames N]\n"
              << "  pmg_cli --inspect-graph graph.pmg\n"
              << "  pmg_cli --inspect-contacts path/to/file.bvh LeftAnkle,RightAnkle\n"
              << "  pmg_cli --export-bvh-recut input.bvh output.bvh first_frame last_frame\n"
              << "  pmg_cli --export-known-cyclic-recuts --bvh-dir BVH --output-dir BVH/recut\n"
              << "  pmg_cli --space-sweep graph_spec.txt node [--contact-joints a,b]\n"
              << "      [--cycle-joint name] [--sweep-steps N] [--min-contacts N]\n"
              << "      [--max-foot-slide X] [--max-adjacent-step X] [--assert-no-regression]\n"
              << "      [--dtw-refine] [--foot-lock]\n"
              << "  pmg_cli --validate-graph graph_spec.txt [--cycle-joint name]\n"
              << "      [--tgood X --tbad Y --source-samples N --target-samples N --seed S]\n"
              << "      [--max-preparation-distance-ratio R]\n"
              << "      [--min-edge-samples N] [--min-good-fraction F] [--assert-no-regression]\n"
              << "      registration options must match explicit GraphSpec settings\n"
              << "  pmg_cli --audit-cyclic-continuity graph_spec.txt\n"
              << "      --output-csv out.csv --output-md out.md [--fps N]\n"
              << "  pmg_cli --audit-root-canonicalization graph_spec.txt\n"
              << "      --output-csv out.csv --output-md out.md\n"
              << "  pmg_cli --audit-registration-phase-alignment graph_spec.txt\n"
              << "      --output-csv out.csv --output-md out.md\n"
              << "  pmg_cli --audit-reachable-region graph.pmg\n"
              << "      --output-csv out.csv --output-md out.md --output-dir out_dir\n"
              << "  pmg_cli --audit-transition-montage graph.pmg\n"
              << "      --output-md out.md --output-dir out_dir\n"
              << "  pmg_cli --probe-transition graph.pmg --source-param x y\n"
              << "      --target-param x y [--frames-before N --frames-after N]\n"
              << "      [--expect-decision accept|reject]\n"
              << "      [--max-root-speed-ratio R --max-yaw-rate-ratio R]\n"
              << "      [--max-contact-drift D --max-foot-height-delta D]\n"
              << "      [--allow-contact-mismatch|--reject-contact-mismatch]\n"
              << "  pmg_cli --audit-contact-transitions graph.pmg\n"
              << "      --output-csv out.csv --output-md out.md [--samples N]\n"
              << "  pmg_cli --audit-transition-acceptance-consistency graph.pmg\n"
              << "      --output-csv out.csv --output-md out.md [--samples N]\n"
              << "  pmg_cli --audit-parameter-response graph.pmg\n"
              << "      --output-csv out.csv --output-md out.md [--samples N]\n"
              << "  pmg_cli --random-walk graph_spec.txt|graph.pmg [--seconds S] [--walk-seed N]\n"
              << "      [--min-transitions N] [--max-pop-ratio X]\n"
              << "      [--blend-placement directional|centered] [--preroll-policy clamp|wrap]\n"
              << "      [--dump-motion-csv out.csv] [--dump-transitions-csv out.csv]\n"
              << "      [--quality-gate --max-root-speed-ratio R --max-yaw-rate-ratio R]\n"
              << "      [--max-contact-drift D --max-foot-height-delta D]\n"
              << "      [--reject-contact-mismatch] (CLI-only prospective ablation;\n"
              << "       core RuntimeController/viewer scheduling unchanged)\n"
              << "      [builder/registration opts]\n"
              << "  pmg_cli --goto graph_spec.txt|graph.pmg x z [--seconds S] [--tolerance D]\n"
              << "      [--facing-degrees DEG --facing-tolerance-degrees DEG]\n"
              << "      [--max-pop-ratio X] [builder/registration opts]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        using CommandRunner = std::optional<int> (*)(int, char**);
        constexpr CommandRunner kCommandRunners[] = {
            pmgcli::TryRunBvhCommand,
            pmgcli::TryRunTransitionCommand,
            pmgcli::TryRunGraphCommand,
            pmgcli::TryRunDiagnosticCommand,
            pmgcli::TryRunRuntimeCommand,
        };
        for (const CommandRunner run_command : kCommandRunners) {
            if (const std::optional<int> exit_code = run_command(argc, argv);
                exit_code.has_value()) {
                return *exit_code;
            }
        }
        PrintUsage();
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
