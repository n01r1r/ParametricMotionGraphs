#include "pmg/BvhLoader.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionSpacePreparation.h"
#include "pmg/RootCanonicalization.h"

#include <cassert>
#include <cmath>
#include <filesystem>

namespace {

bool Near(float a, float b, float tolerance = 1.0e-4f) {
    return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
    std::filesystem::path spec_path("specs/demo_walk_2d_triangulated.pmg_spec");
    if (!std::filesystem::exists(spec_path)) {
        spec_path = "../specs/demo_walk_2d_triangulated.pmg_spec";
    }
    const pmg::GraphSpec spec =
        pmg::LoadGraphSpec(spec_path.string());
    const pmg::PreparedMotionSpaces prepared = pmg::PrepareMotionSpaces(spec);
    const pmg::PreparedMotionSpace& walk = prepared.Node("walk_2d");

    assert(walk.production.NumExamples() == 5);
    for (const pmg::ExampleMotion& example : walk.production.Examples()) {
        const pmg::Pose& first = example.clip.frames.front();
        assert(Near(first.root_position.x, 0.0f));
        assert(Near(first.root_position.z, 0.0f));
        assert(Near(pmg::RootHeadingYaw(first), 0.0f));

        const pmg::Vec3 final_delta =
            example.clip.frames.back().root_position - first.root_position;
        assert(std::sqrt(final_delta.x * final_delta.x +
                         final_delta.z * final_delta.z) > 0.01f);
    }

    return 0;
}
