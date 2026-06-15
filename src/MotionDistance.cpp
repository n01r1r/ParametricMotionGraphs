#include "pmg/MotionDistance.h"
#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <cmath>
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

}  // namespace pmg
