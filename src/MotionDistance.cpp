#include "pmg/MotionDistance.h"
#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace pmg {

namespace {

// Map a (possibly out-of-range) window frame into the clip. Default clamps at
// the endpoints; cyclic_wrap takes it modulo the clip length so a self-edge
// metric sees the real previous/next cycle frame instead of a repeated endpoint.
int ResolveWindowFrame(int frame, int frame_count, bool cyclic_wrap) {
    if (!cyclic_wrap) {
        return std::clamp(frame, 0, frame_count - 1);
    }
    const int wrapped = frame % frame_count;
    return wrapped < 0 ? wrapped + frame_count : wrapped;
}

PointCloud BuildPointCloudFromFirstFrameImpl(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int first_frame,
    int window_size,
    const PointCloudWeighting& weighting,
    bool cyclic_wrap = false) {
    clip.RequireNotEmpty("BuildPointCloudFromFirstFrame");
    if (window_size <= 0) {
        throw std::runtime_error(
            "BuildPointCloudFromFirstFrame: window_size must be positive");
    }
    if (!weighting.per_joint_weights.empty() &&
        static_cast<int>(weighting.per_joint_weights.size()) != skeleton.NumJoints()) {
        throw std::runtime_error(
            "BuildPointCloudFromFirstFrame: per_joint_weights size mismatch");
    }

    const int frame_count = clip.NumFrames();

    PointCloud cloud;
    cloud.points.reserve(static_cast<std::size_t>(window_size) * skeleton.NumJoints());
    cloud.weights.reserve(cloud.points.capacity());

    // Cache world positions per windowed frame so velocity weights can use the
    // forward finite difference between consecutive windowed frames.
    std::vector<std::vector<Vec3>> windowed_positions;
    windowed_positions.reserve(static_cast<std::size_t>(window_size));
    for (int offset = 0; offset < window_size; ++offset) {
        const int frame =
            ResolveWindowFrame(first_frame + offset, frame_count, cyclic_wrap);
        windowed_positions.push_back(ComputeJointWorldPositions(skeleton, clip.frames[frame]));
    }

    for (std::size_t window_index = 0; window_index < windowed_positions.size(); ++window_index) {
        const std::vector<Vec3>& positions = windowed_positions[window_index];
        const std::vector<Vec3>* next_positions =
            window_index + 1 < windowed_positions.size()
                ? &windowed_positions[window_index + 1]
                : nullptr;

        for (std::size_t joint_index = 0; joint_index < positions.size(); ++joint_index) {
            cloud.points.push_back(positions[joint_index]);

            float weight = weighting.per_joint_weights.empty()
                               ? 1.0f
                               : weighting.per_joint_weights[joint_index];
            if (weighting.add_velocity_weight && next_positions != nullptr) {
                const float speed = ((*next_positions)[joint_index] - positions[joint_index]).Norm();
                weight *= (1.0f + speed);
            }
            cloud.weights.push_back(weight);
        }
    }

    return cloud;
}

float RequirePositiveScale(float scale, const char* name) {
    if (scale <= kSmallEpsilon) {
        throw std::runtime_error(
            std::string("TransitionMetricConfig: ") + name +
            " must be positive");
    }
    return scale;
}

float WrappedAngleDelta(float end_angle, float start_angle) {
    float delta = end_angle - start_angle;
    while (delta > kPi) {
        delta -= 2.0f * kPi;
    }
    while (delta < -kPi) {
        delta += 2.0f * kPi;
    }
    return delta;
}

float RootYawRadians(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward = Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

struct ClipKinematics {
    std::vector<std::vector<Vec3>> positions;
    std::vector<std::vector<Vec3>> velocities;
    std::vector<std::vector<Vec3>> accelerations;
    std::vector<float> root_speeds;
    std::vector<float> root_yaws;
    std::vector<float> yaw_rates;
    std::optional<std::vector<std::vector<bool>>> contact_by_joint_frame;
};

std::vector<std::vector<bool>> BuildContactMask(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const std::vector<int>& contact_joint_indices,
    const ContactDetectionSettings& settings) {
    std::vector<std::vector<bool>> mask(
        contact_joint_indices.size(),
        std::vector<bool>(static_cast<std::size_t>(clip.NumFrames()), false));
    const std::vector<ContactInterval> intervals =
        DetectContacts(skeleton, clip, contact_joint_indices, settings);
    for (const ContactInterval& interval : intervals) {
        const auto joint_it = std::find(
            contact_joint_indices.begin(), contact_joint_indices.end(),
            interval.joint_index);
        if (joint_it == contact_joint_indices.end()) {
            continue;
        }
        const std::size_t contact_index =
            static_cast<std::size_t>(
                std::distance(contact_joint_indices.begin(), joint_it));
        const int first_frame = std::max(interval.first_frame, 0);
        const int last_frame = std::min(interval.last_frame, clip.NumFrames() - 1);
        for (int frame = first_frame; frame <= last_frame; ++frame) {
            mask[contact_index][static_cast<std::size_t>(frame)] = true;
        }
    }
    return mask;
}

ClipKinematics PrecomputeKinematics(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const TransitionMetricConfig& metric_config) {
    clip.RequireNotEmpty("PrecomputeKinematics");
    if (clip.NumJoints() != skeleton.NumJoints()) {
        throw std::runtime_error(
            "PrecomputeKinematics: clip/skeleton joint count mismatch");
    }
    if (clip.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "PrecomputeKinematics: frames_per_second must be positive");
    }

    const int frame_count = clip.NumFrames();
    const int joint_count = skeleton.NumJoints();
    const float dt = 1.0f / clip.frames_per_second;

    ClipKinematics kinematics;
    kinematics.positions.reserve(static_cast<std::size_t>(frame_count));
    kinematics.velocities.assign(
        static_cast<std::size_t>(frame_count),
        std::vector<Vec3>(static_cast<std::size_t>(joint_count)));
    kinematics.accelerations.assign(
        static_cast<std::size_t>(frame_count),
        std::vector<Vec3>(static_cast<std::size_t>(joint_count)));
    kinematics.root_speeds.assign(static_cast<std::size_t>(frame_count), 0.0f);
    kinematics.root_yaws.reserve(static_cast<std::size_t>(frame_count));
    kinematics.yaw_rates.assign(static_cast<std::size_t>(frame_count), 0.0f);

    for (const Pose& pose : clip.frames) {
        kinematics.positions.push_back(
            ComputeJointWorldPositions(skeleton, pose));
        kinematics.root_yaws.push_back(RootYawRadians(pose));
    }

    if (frame_count >= 2) {
        for (int frame = 0; frame < frame_count; ++frame) {
            const int previous = std::max(frame - 1, 0);
            const int next = std::min(frame + 1, frame_count - 1);
            const float velocity_dt =
                static_cast<float>(next - previous) * dt;
            for (int joint = 0; joint < joint_count; ++joint) {
                kinematics.velocities[frame][joint] =
                    (kinematics.positions[next][joint] -
                     kinematics.positions[previous][joint]) /
                    velocity_dt;
            }

            const Vec3 root_velocity =
                (clip.frames[next].root_position -
                 clip.frames[previous].root_position) /
                velocity_dt;
            kinematics.root_speeds[frame] =
                std::sqrt(root_velocity.x * root_velocity.x +
                          root_velocity.z * root_velocity.z);
            kinematics.yaw_rates[frame] =
                WrappedAngleDelta(
                    kinematics.root_yaws[next],
                    kinematics.root_yaws[previous]) /
                velocity_dt;
        }
    }

    if (frame_count >= 3) {
        const float acceleration_dt = dt * dt;
        for (int frame = 1; frame + 1 < frame_count; ++frame) {
            for (int joint = 0; joint < joint_count; ++joint) {
                kinematics.accelerations[frame][joint] =
                    (kinematics.positions[frame + 1][joint] -
                     2.0f * kinematics.positions[frame][joint] +
                     kinematics.positions[frame - 1][joint]) /
                    acceleration_dt;
            }
        }
    }

    if (metric_config.contact_settings.has_value() &&
        !metric_config.contact_joint_indices.empty()) {
        for (const int joint_index : metric_config.contact_joint_indices) {
            skeleton.RequireValidJointIndex(
                joint_index, "PrecomputeKinematics contact joint");
        }
        kinematics.contact_by_joint_frame =
            BuildContactMask(
                skeleton, clip, metric_config.contact_joint_indices,
                *metric_config.contact_settings);
    }

    return kinematics;
}

float JointWeight(
    const TransitionMetricConfig& metric_config,
    int joint_index,
    int joint_count) {
    if (metric_config.per_joint_weights.empty()) {
        return 1.0f;
    }
    if (static_cast<int>(metric_config.per_joint_weights.size()) !=
        joint_count) {
        throw std::runtime_error(
            "TransitionMetricConfig: per_joint_weights size mismatch");
    }
    return metric_config.per_joint_weights[static_cast<std::size_t>(joint_index)];
}

float NormalizedRms(float raw_sse, float weight_sum, float scale) {
    if (weight_sum <= kSmallEpsilon) {
        throw std::runtime_error(
            "TransitionMetricConfig: non-positive joint weight sum");
    }
    return std::sqrt(raw_sse / weight_sum) / scale;
}

Vec3 FloorDelta(const Vec3& end, const Vec3& start) {
    return {end.x - start.x, 0.0f, end.z - start.z};
}

}  // namespace

PointCloud MotionDistance::BuildPointCloud(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int center_frame,
    int window_size,
    const PointCloudWeighting& weighting) {
    const int half_window = window_size / 2;
    return BuildPointCloudFromFirstFrameImpl(
        skeleton, clip, center_frame - half_window, window_size, weighting);
}

PointCloud MotionDistance::BuildPointCloudFromFirstFrame(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int first_frame,
    int window_size,
    const PointCloudWeighting& weighting) {
    return BuildPointCloudFromFirstFrameImpl(
        skeleton, clip, first_frame, window_size, weighting);
}

AlignedDistanceResult MotionDistance::AlignedPointCloudDistance(
    const PointCloud& cloud_a,
    const PointCloud& cloud_b) {
    if (cloud_a.Size() != cloud_b.Size()) {
        throw std::runtime_error("MotionDistance::AlignedPointCloudDistance: point count mismatch");
    }
    if (cloud_a.Size() == 0) {
        throw std::runtime_error("MotionDistance::AlignedPointCloudDistance: empty point cloud");
    }

    const std::size_t point_count = cloud_a.Size();

    // Per-point weight: product of both clouds' weights when present, else 1.
    auto point_weight = [&](std::size_t index) -> float {
        float weight = 1.0f;
        if (index < cloud_a.weights.size()) {
            weight *= cloud_a.weights[index];
        }
        if (index < cloud_b.weights.size()) {
            weight *= cloud_b.weights[index];
        }
        return weight;
    };

    // Weighted sums on the floor plane (x,z); y is unaffected by the alignment.
    float total_weight = 0.0f;
    float sum_ax = 0.0f, sum_az = 0.0f;  // weighted sums of cloud A
    float sum_bx = 0.0f, sum_bz = 0.0f;  // weighted sums of cloud B
    float dot_term = 0.0f;               // Sum w (Ax*Bx + Az*Bz)
    float cross_term = 0.0f;             // Sum w (Ax*Bz - Az*Bx)

    for (std::size_t index = 0; index < point_count; ++index) {
        const float weight = point_weight(index);
        const Vec3& a = cloud_a.points[index];
        const Vec3& b = cloud_b.points[index];

        total_weight += weight;
        sum_ax += weight * a.x;
        sum_az += weight * a.z;
        sum_bx += weight * b.x;
        sum_bz += weight * b.z;
        dot_term += weight * (a.x * b.x + a.z * b.z);
        cross_term += weight * (a.x * b.z - a.z * b.x);
    }

    if (total_weight <= kSmallEpsilon) {
        throw std::runtime_error("MotionDistance::AlignedPointCloudDistance: non-positive total weight");
    }

    const float inv_weight = 1.0f / total_weight;
    const float numerator = cross_term - inv_weight * (sum_ax * sum_bz - sum_az * sum_bx);
    const float denominator = dot_term - inv_weight * (sum_ax * sum_bx + sum_az * sum_bz);

    RigidTransform2D alignment;
    alignment.yaw = std::atan2(numerator, denominator);

    const float cos_theta = std::cos(alignment.yaw);
    const float sin_theta = std::sin(alignment.yaw);

    // Rotated cloud-B mean: Ry(theta) applied to (sum_bx, sum_bz).
    const float rotated_bx = cos_theta * sum_bx + sin_theta * sum_bz;
    const float rotated_bz = -sin_theta * sum_bx + cos_theta * sum_bz;
    alignment.dx = inv_weight * (sum_ax - rotated_bx);
    alignment.dz = inv_weight * (sum_az - rotated_bz);

    float weighted_squared = 0.0f;
    for (std::size_t index = 0; index < point_count; ++index) {
        const float weight = point_weight(index);
        const Vec3& a = cloud_a.points[index];
        const Vec3& b = cloud_b.points[index];

        const float aligned_x = cos_theta * b.x + sin_theta * b.z + alignment.dx;
        const float aligned_z = -sin_theta * b.x + cos_theta * b.z + alignment.dz;
        const float delta_x = a.x - aligned_x;
        const float delta_y = a.y - b.y;
        const float delta_z = a.z - aligned_z;
        weighted_squared += weight * (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
    }

    AlignedDistanceResult result;
    result.alignment = alignment;
    result.distance = weighted_squared;
    return result;
}

namespace {

// Normalized phase [0,1] of a frame index within a clip.
float FramePhase(int frame_index, int frame_count) {
    if (frame_count <= 1) {
        return 0.0f;
    }
    return static_cast<float>(frame_index) / static_cast<float>(frame_count - 1);
}

// Sampled frame indices within a normalized phase window, subsampled by stride.
std::vector<int> SampledFrames(int frame_count, float phase_start, float phase_end,
                               int stride) {
    if (frame_count <= 0) {
        throw std::runtime_error("MotionDistance: clip has no frames");
    }
    if (stride < 1) {
        throw std::runtime_error("MotionDistance: frame stride must be >= 1");
    }
    if (phase_start < 0.0f || phase_end > 1.0f || phase_start > phase_end) {
        throw std::runtime_error("MotionDistance: invalid phase window");
    }

    const int last_frame = frame_count - 1;
    const int first = static_cast<int>(std::lround(phase_start * last_frame));
    const int last = static_cast<int>(std::lround(phase_end * last_frame));

    std::vector<int> frames;
    for (int frame = first; frame <= last; frame += stride) {
        frames.push_back(frame);
    }
    if (frames.empty()) {
        frames.push_back(first);
    }
    return frames;
}

}  // namespace

float DistanceGrid::At(int source_index, int target_index) const {
    if (source_index < 0 || source_index >= SourceCount() ||
        target_index < 0 || target_index >= TargetCount()) {
        throw std::runtime_error("DistanceGrid::At: index out of range");
    }
    return distances[static_cast<std::size_t>(source_index) * TargetCount() + target_index];
}

DistanceGrid MotionDistance::BuildDistanceGrid(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& config) {
    return BuildDistanceGridForConvention(
        skeleton, source_clip, target_clip, config,
        TransitionWindowConvention::kKovarDirectional);
}

DistanceGrid MotionDistance::BuildDistanceGridForConvention(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& config,
    TransitionWindowConvention convention) {
    source_clip.RequireNotEmpty("MotionDistance::BuildDistanceGrid source");
    target_clip.RequireNotEmpty("MotionDistance::BuildDistanceGrid target");

    DistanceGrid grid;
    grid.source_frames = SampledFrames(source_clip.NumFrames(),
                                       config.source_phase_start,
                                       config.source_phase_end,
                                       config.source_frame_stride);
    grid.target_frames = SampledFrames(target_clip.NumFrames(),
                                       config.target_phase_start,
                                       config.target_phase_end,
                                       config.target_frame_stride);

    std::vector<PointCloud> source_clouds;
    source_clouds.reserve(grid.source_frames.size());
    for (const int frame : grid.source_frames) {
        const TransitionFrameWindows windows =
            ResolveTransitionFrameWindows(
                source_clip.NumFrames(), target_clip.NumFrames(),
                frame, 0, config.window_size, convention);
        source_clouds.push_back(
            BuildPointCloudFromFirstFrameImpl(
                skeleton, source_clip,
                windows.source.first_unclamped_frame,
                config.window_size,
                config.weighting,
                config.cyclic_wrap));
    }
    std::vector<PointCloud> target_clouds;
    target_clouds.reserve(grid.target_frames.size());
    for (const int frame : grid.target_frames) {
        const TransitionFrameWindows windows =
            ResolveTransitionFrameWindows(
                source_clip.NumFrames(), target_clip.NumFrames(),
                0, frame, config.window_size, convention);
        target_clouds.push_back(
            BuildPointCloudFromFirstFrameImpl(
                skeleton, target_clip,
                windows.target.first_unclamped_frame,
                config.window_size, config.weighting,
                config.cyclic_wrap));
    }

    const std::size_t cell_count = source_clouds.size() * target_clouds.size();
    grid.distances.reserve(cell_count);
    grid.alignments.reserve(cell_count);
    for (const PointCloud& source_cloud : source_clouds) {
        for (const PointCloud& target_cloud : target_clouds) {
            const AlignedDistanceResult result =
                AlignedPointCloudDistance(source_cloud, target_cloud);
            grid.distances.push_back(result.distance);
            grid.alignments.push_back(result.alignment);
        }
    }

    return grid;
}

TransitionMetricResult MotionDistance::EvaluateDynamicsTransition(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    int source_frame,
    int target_frame,
    const DistanceGridConfig& grid_config,
    const TransitionMetricConfig& metric_config,
    TransitionWindowConvention convention) {
    source_clip.RequireNotEmpty("EvaluateDynamicsTransition source");
    target_clip.RequireNotEmpty("EvaluateDynamicsTransition target");
    if (grid_config.window_size <= 0) {
        throw std::runtime_error(
            "EvaluateDynamicsTransition: window_size must be positive");
    }

    const float position_scale =
        RequirePositiveScale(metric_config.position_scale, "position_scale");
    const float velocity_scale =
        RequirePositiveScale(metric_config.velocity_scale, "velocity_scale");
    const float acceleration_scale =
        RequirePositiveScale(
            metric_config.acceleration_scale, "acceleration_scale");
    const float root_speed_scale =
        RequirePositiveScale(metric_config.root_speed_scale, "root_speed_scale");
    const float yaw_rate_scale =
        RequirePositiveScale(metric_config.yaw_rate_scale, "yaw_rate_scale");

    const TransitionFrameWindows windows =
        ResolveTransitionFrameWindows(
            source_clip.NumFrames(), target_clip.NumFrames(),
            source_frame, target_frame, grid_config.window_size, convention);

    PointCloudWeighting alignment_weighting = grid_config.weighting;
    if (alignment_weighting.per_joint_weights.empty()) {
        alignment_weighting.per_joint_weights =
            metric_config.per_joint_weights;
    }
    const PointCloud source_cloud =
        BuildPointCloudFromFirstFrameImpl(
            skeleton, source_clip, windows.source.first_unclamped_frame,
            grid_config.window_size, alignment_weighting,
            grid_config.cyclic_wrap);
    const PointCloud target_cloud =
        BuildPointCloudFromFirstFrameImpl(
            skeleton, target_clip, windows.target.first_unclamped_frame,
            grid_config.window_size, alignment_weighting,
            grid_config.cyclic_wrap);
    const AlignedDistanceResult aligned =
        AlignedPointCloudDistance(source_cloud, target_cloud);

    const ClipKinematics source_kinematics =
        PrecomputeKinematics(skeleton, source_clip, metric_config);
    const ClipKinematics target_kinematics =
        PrecomputeKinematics(skeleton, target_clip, metric_config);

    TransitionMetricResult result;
    result.alignment = aligned.alignment;
    result.compared_frame_count = grid_config.window_size;
    result.compared_joint_count = skeleton.NumJoints();
    result.source_first_frame = windows.source.first_unclamped_frame;
    result.source_last_frame = windows.source.last_unclamped_frame;
    result.target_first_frame = windows.target.first_unclamped_frame;
    result.target_last_frame = windows.target.last_unclamped_frame;

    float joint_weight_sum = 0.0f;
    for (int offset = 0; offset < grid_config.window_size; ++offset) {
        const int source_resolved_frame =
            ResolveWindowFrame(
                windows.source.first_unclamped_frame + offset,
                source_clip.NumFrames(), grid_config.cyclic_wrap);
        const int target_resolved_frame =
            ResolveWindowFrame(
                windows.target.first_unclamped_frame + offset,
                target_clip.NumFrames(), grid_config.cyclic_wrap);

        for (int joint = 0; joint < skeleton.NumJoints(); ++joint) {
            const float weight =
                JointWeight(metric_config, joint, skeleton.NumJoints());
            joint_weight_sum += weight;

            const Vec3 source_position =
                source_kinematics.positions[source_resolved_frame][joint];
            const Vec3 target_position =
                result.alignment.ApplyPoint(
                    target_kinematics.positions[target_resolved_frame][joint]);
            result.raw_position_sse +=
                weight * (source_position - target_position).SquaredNorm();

            const Vec3 source_velocity =
                source_kinematics.velocities[source_resolved_frame][joint];
            const Vec3 target_velocity =
                result.alignment.ApplyVector(
                    target_kinematics.velocities[target_resolved_frame][joint]);
            result.raw_velocity_sse +=
                weight * (source_velocity - target_velocity).SquaredNorm();

            const Vec3 source_acceleration =
                source_kinematics.accelerations[source_resolved_frame][joint];
            const Vec3 target_acceleration =
                result.alignment.ApplyVector(
                    target_kinematics
                        .accelerations[target_resolved_frame][joint]);
            result.raw_acceleration_sse +=
                weight *
                (source_acceleration - target_acceleration).SquaredNorm();
        }

        if (source_kinematics.contact_by_joint_frame.has_value() &&
            target_kinematics.contact_by_joint_frame.has_value()) {
            const auto& source_contacts =
                *source_kinematics.contact_by_joint_frame;
            const auto& target_contacts =
                *target_kinematics.contact_by_joint_frame;
            for (std::size_t contact = 0; contact < source_contacts.size();
                 ++contact) {
                ++result.foot_comparison_count;
                if (source_contacts[contact][source_resolved_frame] !=
                    target_contacts[contact][target_resolved_frame]) {
                    ++result.foot_mismatch_count;
                }
            }
        }
    }

    result.position_cost =
        NormalizedRms(result.raw_position_sse, joint_weight_sum, position_scale);
    result.velocity_cost =
        NormalizedRms(result.raw_velocity_sse, joint_weight_sum, velocity_scale);
    result.acceleration_cost =
        NormalizedRms(
            result.raw_acceleration_sse, joint_weight_sum, acceleration_scale);

    const int source_first =
        ResolveWindowFrame(
            windows.source.first_unclamped_frame, source_clip.NumFrames(),
            grid_config.cyclic_wrap);
    const int source_last =
        ResolveWindowFrame(
            windows.source.last_unclamped_frame, source_clip.NumFrames(),
            grid_config.cyclic_wrap);
    const int target_first =
        ResolveWindowFrame(
            windows.target.first_unclamped_frame, target_clip.NumFrames(),
            grid_config.cyclic_wrap);
    const int target_last =
        ResolveWindowFrame(
            windows.target.last_unclamped_frame, target_clip.NumFrames(),
            grid_config.cyclic_wrap);

    const Vec3 source_root_delta =
        FloorDelta(
            source_clip.frames[source_last].root_position,
            source_clip.frames[source_first].root_position);
    const Vec3 target_root_delta =
        result.alignment.ApplyVector(
            FloorDelta(
                target_clip.frames[target_last].root_position,
                target_clip.frames[target_first].root_position));
    const float root_displacement_cost =
        (source_root_delta - target_root_delta).Norm() / position_scale;

    float root_speed_sse = 0.0f;
    float yaw_rate_sse = 0.0f;
    for (int offset = 0; offset < grid_config.window_size; ++offset) {
        const int source_resolved_frame =
            ResolveWindowFrame(
                windows.source.first_unclamped_frame + offset,
                source_clip.NumFrames(), grid_config.cyclic_wrap);
        const int target_resolved_frame =
            ResolveWindowFrame(
                windows.target.first_unclamped_frame + offset,
                target_clip.NumFrames(), grid_config.cyclic_wrap);
        const float root_speed_delta =
            source_kinematics.root_speeds[source_resolved_frame] -
            target_kinematics.root_speeds[target_resolved_frame];
        root_speed_sse += root_speed_delta * root_speed_delta;

        const float yaw_rate_delta =
            source_kinematics.yaw_rates[source_resolved_frame] -
            target_kinematics.yaw_rates[target_resolved_frame];
        yaw_rate_sse += yaw_rate_delta * yaw_rate_delta;
    }

    const float inv_window =
        1.0f / static_cast<float>(grid_config.window_size);
    const float root_speed_cost =
        std::sqrt(root_speed_sse * inv_window) / root_speed_scale;
    const float yaw_rate_cost =
        std::sqrt(yaw_rate_sse * inv_window) / yaw_rate_scale;
    result.root_cost =
        metric_config.root_displacement_weight * root_displacement_cost +
        metric_config.root_speed_weight * root_speed_cost +
        metric_config.root_yaw_rate_weight * yaw_rate_cost;

    if (result.foot_comparison_count > 0) {
        result.foot_cost =
            metric_config.foot_mismatch_penalty *
            static_cast<float>(result.foot_mismatch_count) /
            static_cast<float>(result.foot_comparison_count);
    }

    result.total_cost =
        metric_config.position_weight * result.position_cost +
        metric_config.velocity_weight * result.velocity_cost +
        metric_config.acceleration_weight * result.acceleration_cost +
        metric_config.root_motion_weight * result.root_cost +
        metric_config.foot_contact_weight * result.foot_cost;
    return result;
}

DistanceGrid MotionDistance::BuildDynamicsDistanceGridForConvention(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& grid_config,
    const TransitionMetricConfig& metric_config,
    TransitionWindowConvention convention) {
    source_clip.RequireNotEmpty("BuildDynamicsDistanceGrid source");
    target_clip.RequireNotEmpty("BuildDynamicsDistanceGrid target");

    DistanceGrid grid;
    grid.source_frames = SampledFrames(source_clip.NumFrames(),
                                       grid_config.source_phase_start,
                                       grid_config.source_phase_end,
                                       grid_config.source_frame_stride);
    grid.target_frames = SampledFrames(target_clip.NumFrames(),
                                       grid_config.target_phase_start,
                                       grid_config.target_phase_end,
                                       grid_config.target_frame_stride);

    const std::size_t cell_count =
        grid.source_frames.size() * grid.target_frames.size();
    grid.distances.reserve(cell_count);
    grid.alignments.reserve(cell_count);
    for (const int source_candidate_frame : grid.source_frames) {
        for (const int target_candidate_frame : grid.target_frames) {
            const TransitionMetricResult result =
                EvaluateDynamicsTransition(
                    skeleton, source_clip, target_clip,
                    source_candidate_frame, target_candidate_frame,
                    grid_config, metric_config, convention);
            grid.distances.push_back(result.total_cost);
            grid.alignments.push_back(result.alignment);
        }
    }
    return grid;
}

OptimalTransition MotionDistance::FindOptimalTransition(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& config,
    float max_distance) {
    return FindOptimalTransitionForConvention(
        skeleton, source_clip, target_clip, config,
        TransitionWindowConvention::kKovarDirectional, max_distance);
}

OptimalTransition MotionDistance::FindOptimalTransitionForConvention(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& config,
    TransitionWindowConvention convention,
    float max_distance) {
    const DistanceGrid grid = BuildDistanceGridForConvention(
        skeleton, source_clip, target_clip, config, convention);

    OptimalTransition best;
    for (int source_index = 0; source_index < grid.SourceCount(); ++source_index) {
        for (int target_index = 0; target_index < grid.TargetCount(); ++target_index) {
            const float distance = grid.At(source_index, target_index);
            if (distance >= best.distance) {
                continue;
            }
            best.distance = distance;
            best.source_frame = grid.source_frames[source_index];
            best.target_frame = grid.target_frames[target_index];
            best.alignment = grid.alignments[
                static_cast<std::size_t>(source_index) * grid.TargetCount() + target_index];
        }
    }

    if (best.source_frame < 0) {
        return best;  // empty grid: valid stays false
    }

    best.source_phase = FramePhase(best.source_frame, source_clip.NumFrames());
    best.target_phase = FramePhase(best.target_frame, target_clip.NumFrames());
    best.valid = best.distance <= max_distance;
    return best;
}

OptimalTransition MotionDistance::FindOptimalDynamicsTransitionForConvention(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& grid_config,
    const TransitionMetricConfig& metric_config,
    TransitionWindowConvention convention,
    float max_distance) {
    const DistanceGrid grid =
        BuildDynamicsDistanceGridForConvention(
            skeleton, source_clip, target_clip, grid_config, metric_config,
            convention);

    OptimalTransition best;
    for (int source_index = 0; source_index < grid.SourceCount(); ++source_index) {
        for (int target_index = 0; target_index < grid.TargetCount(); ++target_index) {
            const float distance = grid.At(source_index, target_index);
            if (distance >= best.distance) {
                continue;
            }
            best.distance = distance;
            best.source_frame = grid.source_frames[source_index];
            best.target_frame = grid.target_frames[target_index];
            best.alignment = grid.alignments[
                static_cast<std::size_t>(source_index) * grid.TargetCount() + target_index];
        }
    }

    if (best.source_frame < 0) {
        return best;
    }

    best.source_phase = FramePhase(best.source_frame, source_clip.NumFrames());
    best.target_phase = FramePhase(best.target_frame, target_clip.NumFrames());
    best.valid = best.distance <= max_distance;
    return best;
}

}  // namespace pmg
