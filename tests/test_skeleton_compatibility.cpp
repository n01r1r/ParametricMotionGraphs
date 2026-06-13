#include "pmg/SkeletonCompatibility.h"

#include <cassert>
#include <stdexcept>

namespace {

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "Root";
    root.parent_index = -1;
    root.channels = {pmg::BvhChannelType::XPosition, pmg::BvhChannelType::YPosition,
                     pmg::BvhChannelType::ZPosition, pmg::BvhChannelType::ZRotation};
    skeleton.joints.push_back(root);

    pmg::Joint child;
    child.name = "Child";
    child.parent_index = 0;
    child.offset = {1.0f, 2.0f, 3.0f};
    child.channels = {pmg::BvhChannelType::XRotation};
    skeleton.joints.push_back(child);
    return skeleton;
}

}  // namespace

int main() {
    const pmg::Skeleton reference = MakeSkeleton();

    pmg::Skeleton same = reference;
    assert(pmg::CheckSkeletonCompatibility(reference, same).compatible);

    pmg::Skeleton renamed = reference;
    renamed.joints[1].name = "Other";
    assert(!pmg::CheckSkeletonCompatibility(reference, renamed).compatible);

    pmg::Skeleton reparented = reference;
    reparented.joints[1].parent_index = -1;
    assert(reparented.NumJoints() == reference.NumJoints());
    assert(!pmg::CheckSkeletonCompatibility(reference, reparented).compatible);

    pmg::Skeleton offset_changed = reference;
    offset_changed.joints[1].offset.x += 0.01f;
    assert(!pmg::CheckSkeletonCompatibility(reference, offset_changed, 1.0e-4f).compatible);

    pmg::Skeleton channels_changed = reference;
    channels_changed.joints[0].channels.push_back(pmg::BvhChannelType::YRotation);
    assert(!pmg::CheckSkeletonCompatibility(reference, channels_changed).compatible);

    bool threw = false;
    try {
        pmg::RequireSkeletonCompatible(reference, renamed, "test");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
    return 0;
}
