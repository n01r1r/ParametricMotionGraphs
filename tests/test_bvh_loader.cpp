#include "pmg/BvhLoader.h"

#include <cassert>
#include <filesystem>
#include <fstream>

int main() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "pmg_minimal_test.bvh";

    std::ofstream output(path);
    output
        << "HIERARCHY\n"
        << "ROOT Hips\n"
        << "{\n"
        << "  OFFSET 0 0 0\n"
        << "  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n"
        << "  JOINT Chest\n"
        << "  {\n"
        << "    OFFSET 1 2 3\n"
        << "    CHANNELS 3 Zrotation Xrotation Yrotation\n"
        << "    End Site ChestTip\n"
        << "    {\n"
        << "      OFFSET 4 5 6\n"
        << "    }\n"
        << "  }\n"
        << "}\n"
        << "MOTION\n"
        << "Frames: 2\n"
        << "Frame Time: 0.0333333\n"
        << "0 0 0 0 0 0 0 0 0\n"
        << "1 0 0 0 0 0 0 0 0\n";
    output.close();

    const pmg::BvhData data = pmg::BvhLoader::Load(path.string());
    // Hips, Chest, and the End Site stored as a zero-channel leaf joint.
    assert(data.skeleton.NumJoints() == 3);
    assert(data.skeleton.joints[2].parent_index == 1);
    assert(data.skeleton.joints[2].name == "ChestTip");
    assert(data.skeleton.joints[2].channels.empty());

    // BVH values are read in explicit x, y, z order in the file's NATIVE units
    // (no unit scaling -- display scale is applied at render time only).
    assert(data.skeleton.joints[1].offset.x == 1.0f);
    assert(data.skeleton.joints[1].offset.y == 2.0f);
    assert(data.skeleton.joints[1].offset.z == 3.0f);
    assert(data.skeleton.joints[2].offset.x == 4.0f);
    assert(data.skeleton.joints[2].offset.y == 5.0f);
    assert(data.skeleton.joints[2].offset.z == 6.0f);

    assert(data.clip.NumFrames() == 2);
    assert(data.clip.frames[1].root_position.x == 1.0f);
    // Each frame pose carries one rotation slot per joint, including the leaf.
    assert(data.clip.frames[0].NumJoints() == 3);

    std::filesystem::remove(path);
    return 0;
}
