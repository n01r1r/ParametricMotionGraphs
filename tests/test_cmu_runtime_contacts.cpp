#include "pmg_test.h"
#include "pmg/PmgOfflinePipeline.h"
#include "pmg/RuntimeController.h"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    for (const char* filename : {"cmu_gait_graph.pmg_spec", "cmu78_gait_graph.pmg_spec"}) {
        const auto artifact = pmg::BuildPmgOfflinePipeline(
            pmg::LoadGraphSpec(std::string(PMG_SOURCE_DIR) + "/specs/" + filename));
        assert(artifact.graph.NumNodes() == 2);
        std::vector<int> feet;
        for (int i=0; i<artifact.skeleton.NumJoints(); ++i) {
            const auto& name = artifact.skeleton.joints[i].name;
            if (name == "LeftFoot" || name == "RightFoot") feet.push_back(i);
        }
        assert(feet.size() == 2);
        pmg::PointCloudAlignment alignment(artifact.skeleton);
        pmg::RuntimeController controller(artifact.graph, alignment);
        const auto initial = artifact.graph.Node(0).motion_space.MaxParameter();
        controller.Start(0, initial, 60);
        controller.EnableFootLocking(artifact.skeleton, feet);
        float max_raw = 0, max_corrected = 0, max_root = 0;
        double raw_squared = 0, corrected_squared = 0;
        int planted = 0;
        bool cross_node = false;
        for (int i=0; i<1200; ++i) {
            const int desired_node = i < 600 ? 1 : 0;
            controller.Update(1.0f/60, {desired_node, artifact.graph.Node(desired_node).motion_space.MaxParameter()});
            cross_node |= controller.CurrentNode() == 1;
            const auto d = controller.FootLockDiagnostics();
            if (d.planted_feet > 0) {
                ++planted;
                max_raw = std::max(max_raw, d.max_raw_anchor_error);
                max_corrected = std::max(max_corrected, d.max_anchor_error);
                raw_squared += d.max_raw_anchor_error * d.max_raw_anchor_error;
                corrected_squared += d.max_anchor_error * d.max_anchor_error;
            }
            max_root = std::max(max_root, d.root_correction);
            assert(std::isfinite(controller.CurrentPose().root_position.x));
        }
        std::cout << filename << ": contacts=" << planted << "/1200, transitions=" << controller.CompletedTransitions()
                  << ", raw/corrected max=" << max_raw << '/' << max_corrected
                  << ", raw/corrected RMS=" << std::sqrt(raw_squared/std::max(1,planted)) << '/'
                  << std::sqrt(corrected_squared/std::max(1,planted)) << ", max pelvis=" << max_root << '\n';
        assert(planted > 0);
        assert(cross_node);
        assert(controller.CompletedTransitions() >= 1);
        // Same detected contacts/anchors, before vs after cleanup. This tests
        // enforcement, not correctness of the heuristic contact labels.
        assert(corrected_squared < raw_squared);
        assert(max_corrected < 0.1f);
    }
}
