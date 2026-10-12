#include "pmg/StreamingFootLock.h"
#include "pmg/FootLocking.h"
#include "Validation.h"

#include <algorithm>
#include <utility>

namespace pmg {

StreamingFootLock::StreamingFootLock(Skeleton skeleton, std::vector<int> joints,
                                   float release_seconds)
    : skeleton_(std::move(skeleton)), joints_(std::move(joints)),
      plants_(joints_.size()), release_seconds_(release_seconds) {
    detail::RequirePositive(release_seconds, "StreamingFootLock release time");
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const int ankle = joints_[i];
        skeleton_.RequireValidJointIndex(ankle, "StreamingFootLock ankle");
        const int knee = skeleton_.joints[ankle].parent_index;
        skeleton_.RequireValidJointIndex(knee, "StreamingFootLock knee");
        const int hip = skeleton_.joints[knee].parent_index;
        skeleton_.RequireValidJointIndex(hip, "StreamingFootLock hip");
        if (skeleton_.joints[ankle].offset.Norm() <= kSmallEpsilon ||
            skeleton_.joints[knee].offset.Norm() <= kSmallEpsilon) {
            throw std::runtime_error("StreamingFootLock: zero-length leg bone");
        }
        for (std::size_t j = 0; j < i; ++j) {
            const int other_knee = skeleton_.joints[joints_[j]].parent_index;
            const int other_hip = skeleton_.joints[other_knee].parent_index;
            if (ankle == joints_[j] || knee == other_knee || hip == other_hip) {
                throw std::runtime_error("StreamingFootLock: legs must have independent IK chains");
            }
        }
    }
}

void StreamingFootLock::Reset() {
    plants_.assign(joints_.size(), Plant{});
    root_offset_ = {};
    diagnostics_ = {};
}

Pose StreamingFootLock::Update(const Pose& raw_pose, const std::vector<bool>& contacts, float dt) {
    detail::RequireFinite(dt, "StreamingFootLock dt");
    if (dt < 0.0f || contacts.size() != joints_.size()) {
        throw std::runtime_error("StreamingFootLock: invalid dt or contact count");
    }
    const auto raw = ComputeForwardKinematics(skeleton_, raw_pose);
    diagnostics_ = {};
    std::vector<Vec3> targets(joints_.size());
    std::vector<float> weights(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        Plant& plant = plants_[i];
        const auto& ankle = raw[joints_[i]];
        if (contacts[i]) {
            if (!plant.active) {
                plant.anchor = ankle.position + root_offset_;
                plant.orientation = ankle.rotation;
            }
            plant.active = true;
            plant.release_weight = 1.0f;
            ++diagnostics_.planted_feet;
        } else {
            if (plant.active) plant.release_offset = plant.anchor - ankle.position;
            plant.active = false;
            plant.release_weight = std::max(0.0f, plant.release_weight - dt / release_seconds_);
        }
        const float t = plant.release_weight;
        weights[i] = t * t * (3.0f - 2.0f * t);
        targets[i] = plant.active ? plant.anchor : ankle.position + plant.release_offset * weights[i];
    }

    // Project the pelvis into the intersection of the active legs' reachable
    // balls. Bounded iterations preserve fixed bone lengths and avoid an
    // unbounded solve when constraints conflict. Remaining error is reported.
    Vec3 correction;
    for (int iteration = 0; iteration < 8; ++iteration) {
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            if (!plants_[i].active) continue;
            const int knee = skeleton_.joints[joints_[i]].parent_index;
            const int hip = skeleton_.joints[knee].parent_index;
            const float reach = (skeleton_.joints[knee].offset.Norm() +
                                 skeleton_.joints[joints_[i]].offset.Norm()) * (1.0f - 1.0e-5f);
            const Vec3 delta = targets[i] - (raw[hip].position + correction);
            const float distance = delta.Norm();
            if (distance > reach) correction = correction + delta * ((distance - reach) / distance);
        }
    }
    if (diagnostics_.planted_feet == 0) {
        root_offset_ = root_offset_ * std::exp(-dt / release_seconds_);
    } else {
        root_offset_ = correction;
    }
    Pose pose = raw_pose;
    pose.root_position = pose.root_position + root_offset_;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (weights[i] <= 0.0f) continue;
        SolveTwoBoneIk(skeleton_, pose, joints_[i], targets[i]);
        const auto world = ComputeForwardKinematics(skeleton_, pose);
        const int knee = skeleton_.joints[joints_[i]].parent_index;
        const auto orientation = Slerp(raw[joints_[i]].rotation, plants_[i].orientation, weights[i]);
        pose.local_rotations[joints_[i]] =
            (world[knee].rotation.Conjugate() * orientation).Normalized();
    }
    const auto corrected = ComputeForwardKinematics(skeleton_, pose);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (plants_[i].active) {
            diagnostics_.max_raw_anchor_error = std::max(diagnostics_.max_raw_anchor_error,
                (raw[joints_[i]].position - plants_[i].anchor).Norm());
            diagnostics_.max_anchor_error = std::max(diagnostics_.max_anchor_error,
                (corrected[joints_[i]].position - plants_[i].anchor).Norm());
        }
    }
    diagnostics_.root_correction = root_offset_.Norm();
    return pose;
}

} // namespace pmg
