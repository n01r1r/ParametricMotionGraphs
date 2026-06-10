#include "pmg/MotionDistance.h"
#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

float MotionDistance::PoseDistance(
    const Skeleton& skeleton,
    const Pose& source_pose,
    const Pose& target_pose) {
    source_pose.RequireJointCount(skeleton.NumJoints(), "MotionDistance::PoseDistance source");
    target_pose.RequireJointCount(skeleton.NumJoints(), "MotionDistance::PoseDistance target");

    const std::vector<Vec3> source_positions = ComputeJointWorldPositions(skeleton, source_pose);
    const std::vector<Vec3> target_positions = ComputeJointWorldPositions(skeleton, target_pose);

    if (source_positions.empty()) {
        throw std::runtime_error("MotionDistance::PoseDistance: skeleton must contain at least one joint");
    }

    const Vec3 root_alignment_delta = source_positions.front() - target_positions.front();

    float sum_squared_distance = 0.0f;
    for (std::size_t joint_index = 0; joint_index < source_positions.size(); ++joint_index) {
        const Vec3 aligned_target = target_positions[joint_index] + root_alignment_delta;
        sum_squared_distance += (source_positions[joint_index] - aligned_target).SquaredNorm();
    }

    return sum_squared_distance / static_cast<float>(source_positions.size());
}

PointCloud MotionDistance::BuildPointCloud(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int center_frame,
    int window_size,
    const PointCloudWeighting& weighting) {
    clip.RequireNotEmpty("MotionDistance::BuildPointCloud");
    if (window_size <= 0) {
        throw std::runtime_error("MotionDistance::BuildPointCloud: window_size must be positive");
    }
    if (!weighting.per_joint_weights.empty() &&
        static_cast<int>(weighting.per_joint_weights.size()) != skeleton.NumJoints()) {
        throw std::runtime_error("MotionDistance::BuildPointCloud: per_joint_weights size mismatch");
    }

    const int frame_count = clip.NumFrames();
    const int half_window = window_size / 2;

    PointCloud cloud;
    cloud.points.reserve(static_cast<std::size_t>(window_size) * skeleton.NumJoints());
    cloud.weights.reserve(cloud.points.capacity());

    // Cache world positions per windowed frame so velocity weights can use the
    // forward finite difference between consecutive windowed frames.
    std::vector<std::vector<Vec3>> windowed_positions;
    windowed_positions.reserve(static_cast<std::size_t>(window_size));
    for (int offset = 0; offset < window_size; ++offset) {
        const int frame = std::clamp(center_frame - half_window + offset, 0, frame_count - 1);
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

    RigidAlignment2D alignment;
    alignment.theta = std::atan2(numerator, denominator);

    const float cos_theta = std::cos(alignment.theta);
    const float sin_theta = std::sin(alignment.theta);

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
    result.distance = weighted_squared * inv_weight;
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

    // Build each frame's point cloud once, then fill the grid.
    std::vector<PointCloud> source_clouds;
    source_clouds.reserve(grid.source_frames.size());
    for (const int frame : grid.source_frames) {
        source_clouds.push_back(
            BuildPointCloud(skeleton, source_clip, frame, config.window_size, config.weighting));
    }
    std::vector<PointCloud> target_clouds;
    target_clouds.reserve(grid.target_frames.size());
    for (const int frame : grid.target_frames) {
        target_clouds.push_back(
            BuildPointCloud(skeleton, target_clip, frame, config.window_size, config.weighting));
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

OptimalTransition MotionDistance::FindOptimalTransition(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& config,
    float max_distance) {
    const DistanceGrid grid = BuildDistanceGrid(skeleton, source_clip, target_clip, config);

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

TransitionMatch MotionDistance::FindBestTransition(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const TransitionSearchConfig& config) {
    source_clip.RequireNotEmpty("MotionDistance::FindBestTransition source");
    target_clip.RequireNotEmpty("MotionDistance::FindBestTransition target");

    if (config.samples_per_window <= 0) {
        throw std::runtime_error("MotionDistance::FindBestTransition: samples_per_window must be positive");
    }

    TransitionMatch best_match;

    for (int source_sample = 0; source_sample < config.samples_per_window; ++source_sample) {
        const float source_alpha =
            config.samples_per_window == 1
                ? 0.0f
                : static_cast<float>(source_sample) / static_cast<float>(config.samples_per_window - 1);
        const float source_phase =
            (1.0f - source_alpha) * config.source_phase_start + source_alpha * config.source_phase_end;
        const Pose source_pose = source_clip.SampleNormalizedPhase(source_phase);

        for (int target_sample = 0; target_sample < config.samples_per_window; ++target_sample) {
            const float target_alpha =
                config.samples_per_window == 1
                    ? 0.0f
                    : static_cast<float>(target_sample) / static_cast<float>(config.samples_per_window - 1);
            const float target_phase =
                (1.0f - target_alpha) * config.target_phase_start + target_alpha * config.target_phase_end;
            const Pose target_pose = target_clip.SampleNormalizedPhase(target_phase);

            const float distance = PoseDistance(skeleton, source_pose, target_pose);
            if (distance < best_match.distance) {
                best_match.distance = distance;
                best_match.source_phase = source_phase;
                best_match.target_phase = target_phase;
            }
        }
    }

    return best_match;
}

}  // namespace pmg
