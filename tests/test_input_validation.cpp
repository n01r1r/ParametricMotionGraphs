#include "pmg_test.h"
#include "pmg/BvhLoader.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/MotionClip.h"
#include "pmg/PoseBlend.h"
#include "pmg/TimeWarp.h"
#include "pmg/RuntimeController.h"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <limits>

namespace {
template<class F> void Rejects(F action) {
    bool threw = false;
    try { action(); } catch (const std::exception&) { threw = true; }
    assert(threw);
}
}
int main() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    pmg::Pose pose; pose.local_rotations.resize(1);
    pmg::MotionClip clip; clip.frames = {pose, pose}; clip.frames_per_second = 30;
    Rejects([&] { clip.SampleByFrame(nan); });
    Rejects([&] { clip.SampleNormalizedPhase(nan); });
    Rejects([&] { pmg::TimeWarp::FromAnchors({nan}, {0.5f}); });
    Rejects([&] { pmg::TimeWarp{}.Evaluate(nan); });
    Rejects([&] { pmg::BlendPose(pose, pose, nan); });
    Rejects([&] { pmg::BlendPoseN({pose, pose}, {nan, 1}); });
    Rejects([&] { pmg::BlendPoseN({pose, pose}, {inf, 1}); });
    auto translated = pose; translated.root_position.x = 10;
    assert(std::abs(pmg::BlendPoseN({pose, translated}, {1e38f, 1e38f}).root_position.x - 5) < 1e-5f);
    assert(std::abs(pmg::BlendPoseN({pose, translated}, {1e-20f, 1e-20f}).root_position.x - 5) < 1e-5f);
    pmg::ParameterAabb box{{0}, {1}};
    assert(!box.Contains({nan}));
    assert(!pmg::ParameterAabb{}.Contains({}));
    Rejects([&] { pmg::ParameterAabb{}.ShrinkToExclude({}, 0); });
    Rejects([&] { pmg::ParameterAabb::Empty(-1); });
    Rejects([&] { pmg::ParameterAabb{{1}, {0}}.Clamp({0}); });
    pmg::Skeleton skeleton; skeleton.joints.resize(1); skeleton.joints[0].parent_index = 0;
    Rejects([&] { pmg::ComputeForwardKinematics(skeleton, pose); });
    skeleton.joints[0].parent_index = -2;
    Rejects([&] { pmg::ComputeForwardKinematics(skeleton, pose); });
    pmg::ParametricMotionSpace space("test", 1); space.AddExample({0}, clip); space.AddExample({1}, clip);
    Rejects([&] { space.GenerateClip({nan}, 30); });
    Rejects([&] { space.GenerateClip({0}, inf); });
    Rejects([&] { space.GenerateClip({0}, 1e30f); });
    pmg::ParametricMotionGraph graph; graph.AddNode("test", space);
    pmg::RootOnlyAlignment alignment; pmg::RuntimeController controller(graph, alignment);
    controller.Start(0, {2}, 30);
    assert(controller.CurrentParameter()[0] == 1);
    Rejects([&] { controller.Update(inf, {}); });
    Rejects([&] { controller.Update(nan, {}); });
    Rejects([&] { controller.Start(0, {nan}, 30); });
    assert(controller.CurrentParameter()[0] == 1);
    // Previously looped one iteration per completed cycle.
    controller.Update(1e7f, {});
    assert(std::isfinite(controller.CurrentPose().root_position.x));
    const auto path = std::filesystem::temp_directory_path() / "pmg_invalid_token.bvh";
    auto write = [&](const char* frame) {
        std::ofstream out(path);
        out << "HIERARCHY ROOT Hips { OFFSET 0 0 0 CHANNELS 1 Xposition } MOTION Frames: 1 Frame Time: 0.03 " << frame;
    };
    for (const char* token : {"1junk", "nan", "inf", "0 1"}) {
        write(token); Rejects([&] { pmg::BvhLoader::Load(path.string()); });
    }
    std::filesystem::remove(path);
}
