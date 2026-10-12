#include "pmg/RuntimeController.h"
#include "Validation.h"

#include <algorithm>
#include <cmath>

namespace pmg {

void RuntimeController::EnableFootLocking(const Skeleton& skeleton,
                                        const std::vector<int>& joints) {
    foot_lock_.emplace(skeleton, joints);
    corrected_pose_.reset();
    if (current_node_ >= 0) {
        current_contacts_ = PrepareFootContacts(current_clip_, graph_.Node(current_node_).motion_space, current_parameter_);
        next_contacts_ = transition_active_ ? PrepareFootContacts(next_clip_, graph_.Node(next_node_).motion_space, next_parameter_) : std::vector<ContactInterval>{};
        RefreshFootPose(0.0f);
    }
}

void RuntimeController::DisableFootLocking() {
    foot_lock_.reset();
    corrected_pose_.reset();
    current_contacts_.clear();
    next_contacts_.clear();
}

FootPlantDiagnostics RuntimeController::FootLockDiagnostics() const {
    return foot_lock_ ? foot_lock_->Diagnostics() : FootPlantDiagnostics{};
}

std::vector<ContactInterval> RuntimeController::PrepareFootContacts(
    const MotionClip& clip, const ParametricMotionSpace& space,
    const ParameterVector& parameter) const {
    std::vector<ContactInterval> result;
    if (!foot_lock_) return result;
    const auto weights = space.ComputeLocalBlendWeights(parameter);
    // Infer labels on original examples, then apply the SAME phase warps and
    // blend weights as synthesis. Detecting only on an already sliding blend
    // would incorrectly label a fast-skidding stance foot as airborne.
    for (int joint : foot_lock_->Joints()) {
        std::vector<std::vector<ContactInterval>> tracks;
        for (const auto& example : space.Examples()) {
            const auto settings = EstimateContactSettings(foot_lock_->GetSkeleton(), example.clip, {joint});
            tracks.push_back(DetectContacts(foot_lock_->GetSkeleton(), example.clip, {joint}, settings));
        }
        int first = -1;
        for (int frame = 0; frame <= clip.NumFrames(); ++frame) {
            float confidence = 0.0f;
            if (frame < clip.NumFrames()) {
                const float phase = static_cast<float>(frame) / std::max(1, clip.NumFrames() - 1);
                for (std::size_t e = 0; e < tracks.size(); ++e) {
                    const float warped = space.HasExampleTimeWarps() ? space.ExampleTimeWarps()[e].Evaluate(phase) : phase;
                    const int sample = static_cast<int>(std::lround(warped * (space.Examples()[e].clip.NumFrames() - 1)));
                    if (std::any_of(tracks[e].begin(), tracks[e].end(), [&](const auto& interval) {
                        return sample >= interval.first_frame && sample <= interval.last_frame;
                    })) confidence += weights[e];
                }
            }
            if (confidence >= 0.5f && first < 0) first = frame;
            if (confidence < 0.5f && first >= 0) {
                result.push_back({joint, first, frame - 1});
                first = -1;
            }
        }
    }
    return result;
}

void RuntimeController::RefreshFootPose(float dt) {
    if (!foot_lock_) return;
    const auto contact_at = [](const std::vector<ContactInterval>& intervals,
                               int joint, float phase, int frames) {
        const int frame = static_cast<int>(std::lround(phase * (frames - 1)));
        return std::any_of(intervals.begin(), intervals.end(), [&](const auto& interval) {
            return interval.joint_index == joint && frame >= interval.first_frame && frame <= interval.last_frame;
        });
    };
    std::vector<bool> contacts;
    for (int joint : foot_lock_->Joints()) {
        bool contact = contact_at(current_contacts_, joint, CurrentPhase(), current_clip_.NumFrames());
        // The dominant clip supplies the contact label during a cross-fade.
        // If both clips agree on stance, the same world anchor survives it.
        if (transition_active_ && transition_elapsed_seconds_ >= 0.5f * transition_duration_seconds_) {
            contact = contact_at(next_contacts_, joint, ClipPhase(next_clip_, next_time_seconds_), next_clip_.NumFrames());
        }
        contacts.push_back(contact);
    }
    corrected_pose_ = foot_lock_->Update(RawPose(), contacts, dt);
}

Pose RuntimeController::CurrentPose() const {
    return corrected_pose_ ? *corrected_pose_ : RawPose();
}

void RuntimeController::Update(float dt, const RuntimeControlRequest& request) {
    detail::RequireFinite(dt, "RuntimeController::Update dt");
    if (dt < 0.0f) throw std::runtime_error("RuntimeController::Update: negative dt");
    if (!foot_lock_ || dt == 0.0f) {
        Advance(dt, request);
        RefreshFootPose(dt);
        return;
    }
    // Evaluate contact events at least at the animation sampling rate rather
    // than skipping stance phases when rendering runs below the clip's FPS.
    // A suspension/seek over one second invalidates old world anchors.
    if (dt > 1.0f) {
        foot_lock_->Reset();
        Advance(dt, request);
        RefreshFootPose(0.0f);
        return;
    }
    const int steps = std::max(1, static_cast<int>(std::min(240.0, std::ceil(static_cast<double>(dt) * frames_per_second_))));
    const float step = dt / static_cast<float>(steps);
    for (int i = 0; i < steps; ++i) {
        Advance(step, request);
        RefreshFootPose(step);
    }
}

} // namespace pmg
