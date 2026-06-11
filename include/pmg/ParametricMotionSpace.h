#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParameterVector.h"
#include "pmg/ParameterDomain.h"
#include "pmg/TimeWarp.h"

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

// Measurable motion property used to make blends parameter-accurate.
// kTurnRate: signed mean root-heading rate in radians per second.
enum class ParameterMetric { kNone = 0, kTurnRate = 1 };

struct CalibrationSample {
    float blend_t = 0.0f;   // weight on the segment's right example
    float measured = 0.0f;  // metric value of the blend generated at blend_t
};

// Sampled measured-metric curve between two parameter-adjacent examples.
struct CalibrationSegment {
    int left_example = -1;
    int right_example = -1;
    std::vector<CalibrationSample> samples;  // ascending blend_t, includes 0 and 1
};

// Kovar-Gleicher 2004 style accuracy table for a one-dimensional space:
// requested parameter -> target measured value (piecewise-linear over the
// examples' measured anchors) -> the blend weight that actually achieves it
// (inverted from the sampled curve). Without it, weights are Shepard
// interpolation and the parameter is only approximately achieved.
struct ParameterCalibration {
    ParameterMetric metric = ParameterMetric::kNone;
    std::vector<int> example_order;       // example indices sorted by parameter
    std::vector<float> example_measured;  // measured metric per sorted example
    std::vector<CalibrationSegment> segments;  // one per adjacent sorted pair
};

// Measured metric of one generated clip (kTurnRate: net wrapped root-heading
// change divided by clip duration).
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

    // Calibration makes 1-D blends parameter-accurate: when present,
    // ComputeLocalBlendWeights inverts the measured-metric table instead of
    // assuming the parameter is linear in the weights.
    void SetParameterCalibration(ParameterCalibration calibration);
    void ClearParameterCalibration();
    bool HasParameterCalibration() const;
    const ParameterCalibration& ParameterCalibrationData() const;

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
    // metric == kNone means uncalibrated (Shepard weights).
    ParameterCalibration parameter_calibration_;

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

// Offline calibration: for every parameter-adjacent example pair, sample the
// blend weight t in [0, 1], generate each blend, and measure the metric. The
// sampled curve is forced monotone toward its endpoint so the runtime
// inversion is well defined. Requires a one-dimensional space.
ParameterCalibration CalibrateParameterMetric(
    const ParametricMotionSpace& space,
    ParameterMetric metric,
    float frames_per_second,
    int samples_per_segment = 9);

}  // namespace pmg
