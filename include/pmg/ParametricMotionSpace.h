#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParameterVector.h"
#include "pmg/ParameterDomain.h"
#include "pmg/ParameterSupport.h"
#include "pmg/TimeWarp.h"

#include <optional>
#include <string>
#include <vector>

namespace pmg {

class ParametricMotionSpace;

namespace legacy {
MotionClip GenerateClipWithFrameCount(
    const ParametricMotionSpace& space,
    const ParameterVector& parameter,
    int frame_count,
    float frames_per_second);
}  // namespace legacy

struct ExampleMotion {
    ParameterVector parameter;
    MotionClip clip;
};

// Measurable motion properties used to make blends parameter-accurate.
// kTurnRate: signed mean root-heading rate in radians per second.
// kTravelSpeed: mean root path speed in native BVH distance units per second.
enum class ParameterMetric {
    kNone = 0,
    kTurnRate = 1,
    kTravelSpeed = 2,
};

// One sampled inverse-calibration point. measured_parameter has one value per
// configured metric. blend_weights has one value per authored example and
// sums to one.
struct CalibrationSample {
    ParameterVector measured_parameter;
    std::vector<float> blend_weights;
};

// Kovar-Gleicher style sampled inverse map:
// requested authored coordinate -> anchor-interpolated measured coordinate ->
// blend weights that achieve that coordinate. Metric distances are normalized
// by metric_scales so unlike units (for example rad/s and BVH units/s) remain
// comparable during local inversion.
struct ParameterCalibration {
    std::vector<ParameterMetric> metrics;  // exactly one metric per parameter axis
    std::vector<ParameterVector> example_measured;  // indexed by authored example
    ParameterVector metric_scales;  // positive normalization scale per metric
    std::vector<CalibrationSample> samples;
    int samples_per_axis = 0;  // 0 for imported/manual tables
};

// Measured property of one generated clip. Throws for kNone or an unsupported
// metric.
float MeasureParameterMetric(ParameterMetric metric, const MotionClip& clip);

class ParametricMotionSpace {
public:
    ParametricMotionSpace() = default;
    ParametricMotionSpace(std::string space_name, int parameter_dimension);

    const std::string& Name() const;
    int ParameterDimension() const;
    int NumExamples() const;

    void AddExample(const ParameterVector& parameter, MotionClip clip);

    // Registration warps, one per example, mapping the space's canonical
    // phase onto each example's own phase (see MotionRegistration). When set,
    // EvaluatePose samples example i at warps[i](phase) so blends combine
    // structurally corresponding moments instead of the same raw phase.
    void SetExampleTimeWarps(std::vector<TimeWarp> warps);
    void ClearExampleTimeWarps();
    bool HasExampleTimeWarps() const;
    // Empty when unregistered; otherwise one warp per example.
    const std::vector<TimeWarp>& ExampleTimeWarps() const;

    // Calibration makes multidimensional blends parameter-accurate: when
    // present, ComputeLocalBlendWeights inverts the sampled measured-parameter
    // map instead of assuming authored coordinates are linear in blend weights.
    void SetParameterCalibration(ParameterCalibration calibration);
    void ClearParameterCalibration();
    bool HasParameterCalibration() const;
    const ParameterCalibration& ParameterCalibrationData() const;

    void SetParameterSupport(ParameterSupport support);
    const std::optional<ParameterSupport>& ExplicitSupport() const;
    bool HasExplicitParameterSupport() const;

    ParameterVector ProjectToSupport(const ParameterVector& parameter) const;
    bool ContainsSupportedParameter(const ParameterVector& parameter) const;
    ParameterVector SampleSupportedParameter(std::mt19937& rng) const;

    std::vector<float> ComputeLocalBlendWeights(
        const ParameterVector& parameter) const;

    Pose EvaluatePose(const ParameterVector& parameter, float normalized_phase) const;

    // Blended cycle duration: weighted sum of the examples' clip durations.
    // The paper's blends inherit timing from their examples, so a tight turn
    // and a straight walk must not share one fixed frame count.
    float BlendedDurationSeconds(const ParameterVector& parameter) const;

    // Generates a clip by blending joints/height per frame (EvaluatePose) but
    // integrating the root's floor motion from blended per-frame root deltas
    // expressed in each example's own heading frame. Blending absolute root
    // positions would average arcs of different curvature into a distorted
    // path and drag planted feet sideways; delta integration follows an
    // intermediate arc instead.
    //
    // Paper path: frame count derives from BlendedDurationSeconds.
    MotionClip GenerateClip(
        const ParameterVector& parameter,
        float frames_per_second) const;

    std::vector<ParameterVector> ExampleParameters() const;
    const std::vector<ExampleMotion>& Examples() const;
    std::vector<float> MinParameter() const;
    std::vector<float> MaxParameter() const;
    ParameterDomain Domain() const;
    ParameterVector ClampToDomain(const ParameterVector& parameter) const;

private:
    std::vector<float> CalibratedBlendWeights(const ParameterVector& parameter) const;
    std::vector<float> UncalibratedBlendWeights(
        const ParameterVector& parameter) const;
    Pose EvaluatePoseFromWeights(
        const std::vector<float>& weights, float normalized_phase) const;
    MotionClip GenerateClipFromWeights(
        const std::vector<float>& weights,
        int frame_count,
        float frames_per_second) const;

    std::string name_ = "unnamed_space";
    int parameter_dimension_ = 0;
    std::vector<ExampleMotion> examples_;
    // Empty = unregistered (every example sampled at the raw phase).
    std::vector<TimeWarp> example_time_warps_;
    // Empty metrics means uncalibrated (Shepard weights).
    ParameterCalibration parameter_calibration_;
    std::optional<ParameterSupport> parameter_support_;

    friend ParameterCalibration CalibrateParameterMetrics(
        const ParametricMotionSpace& space,
        const std::vector<ParameterMetric>& metrics,
        float frames_per_second,
        int samples_per_axis);
    friend ParameterCalibration CalibrateParameterMetric(
        const ParametricMotionSpace& space,
        ParameterMetric metric,
        float frames_per_second,
        int samples_per_segment);
    friend MotionClip legacy::GenerateClipWithFrameCount(
        const ParametricMotionSpace& space,
        const ParameterVector& parameter,
        int frame_count,
        float frames_per_second);
};

// Offline multidimensional calibration. Samples a deterministic regular grid
// over the authored parameter domain, generates each uncalibrated blend, and
// records the resulting metric vector and full example-weight vector.
// metrics.size() must equal the space parameter dimension.
ParameterCalibration CalibrateParameterMetrics(
    const ParametricMotionSpace& space,
    const std::vector<ParameterMetric>& metrics,
    float frames_per_second,
    int samples_per_axis = 9);

// Compatibility wrapper for one-dimensional callers.
ParameterCalibration CalibrateParameterMetric(
    const ParametricMotionSpace& space,
    ParameterMetric metric,
    float frames_per_second,
    int samples_per_segment = 9);

}  // namespace pmg
