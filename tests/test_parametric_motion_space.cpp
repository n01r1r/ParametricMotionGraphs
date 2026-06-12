#include "pmg/ParametricMotionSpace.h"

#include <cassert>
#include <cmath>

namespace {

pmg::MotionClip MakeClip(float root_x) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;

    pmg::Pose pose;
    pose.root_position = {root_x, 0.0f, 0.0f};
    pose.local_rotations.push_back(pmg::Quaternion::Identity());
    clip.frames.push_back(pose);
    return clip;
}

pmg::MotionClip MakeAdvancingClip(float root_x, int frame_count) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    for (int frame = 0; frame < frame_count; ++frame) {
        const float phase =
            static_cast<float>(frame) / static_cast<float>(frame_count - 1);
        pmg::Pose pose;
        pose.root_position = {root_x, 0.0f, 3.0f * phase};
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::MotionClip MakeLocomotionClip(
    float turn_degrees,
    float travel_distance,
    int frame_count) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    for (int frame = 0; frame < frame_count; ++frame) {
        const float phase =
            static_cast<float>(frame) / static_cast<float>(frame_count - 1);
        pmg::Pose pose;
        pose.root_position = {0.0f, 0.0f, travel_distance * phase};
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('Y', turn_degrees * phase));
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::MotionClip MakeTurningClip(float turn_degrees, int frame_count) {
    return MakeLocomotionClip(turn_degrees, 3.0f, frame_count);
}

}  // namespace

int main() {
    pmg::ParametricMotionSpace space("test", 1);
    space.AddExample({0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f}, MakeClip(10.0f));

    const pmg::Pose exact = space.EvaluatePose({1.0f}, 0.0f);
    assert(std::abs(exact.root_position.x - 10.0f) < 1.0e-5f);

    const pmg::Pose middle = space.EvaluatePose({0.5f}, 0.0f);
    assert(middle.root_position.x > 0.0f);
    assert(middle.root_position.x < 10.0f);

    // D2: blended cycle duration follows the examples' durations, and the
    // duration-derived GenerateClip overload honors it.
    {
        pmg::ParametricMotionSpace timed("timed", 1);
        timed.AddExample({0.0f}, MakeAdvancingClip(0.0f, 31));   // 1.0 s
        timed.AddExample({1.0f}, MakeAdvancingClip(10.0f, 61));  // 2.0 s
        assert(std::abs(timed.BlendedDurationSeconds({0.0f}) - 1.0f) < 1.0e-4f);
        assert(std::abs(timed.BlendedDurationSeconds({1.0f}) - 2.0f) < 1.0e-4f);
        const float middle_duration = timed.BlendedDurationSeconds({0.5f});
        assert(middle_duration > 1.0f && middle_duration < 2.0f);

        const pmg::MotionClip generated = timed.GenerateClip({0.5f}, 30.0f);
        const float generated_duration = generated.DurationSeconds();
        assert(std::abs(generated_duration - middle_duration) < 0.05f);
    }

    // D1 inversion: a hand-built nonlinear calibration table redirects the
    // requested parameter to the blend weight that achieves the anchored
    // measured value, instead of assuming parameter ~ weight.
    {
        pmg::ParametricMotionSpace calibrated("calibrated", 1);
        calibrated.AddExample({0.0f}, MakeClip(0.0f));
        calibrated.AddExample({1.0f}, MakeClip(10.0f));

        pmg::ParameterCalibration table;
        table.metrics = {pmg::ParameterMetric::kTurnRate};
        table.example_measured = {{0.0f}, {1.0f}};
        table.metric_scales = {1.0f};
        // Measured rises fast early: m(0.5) = 0.8. Requesting the parameter
        // midpoint (target m = 0.5) must therefore choose t < 0.5.
        table.samples = {
            {{0.0f}, {1.0f, 0.0f}},
            {{0.8f}, {0.5f, 0.5f}},
            {{1.0f}, {0.0f, 1.0f}},
        };
        calibrated.SetParameterCalibration(table);

        const std::vector<float> weights =
            calibrated.ComputeLocalBlendWeights({0.5f});
        const float expected_t = 0.5f * (0.5f / 0.8f);  // 0.3125
        assert(std::abs(weights[1] - expected_t) < 1.0e-4f);
        assert(std::abs(weights[0] - (1.0f - expected_t)) < 1.0e-4f);

        // Endpoints still hit the examples exactly.
        assert(std::abs(calibrated.ComputeLocalBlendWeights({0.0f})[0] - 1.0f) <
               1.0e-4f);
        assert(std::abs(calibrated.ComputeLocalBlendWeights({1.0f})[1] - 1.0f) <
               1.0e-4f);
    }

    // D1 end-to-end: calibrate a turning space against the turn-rate metric;
    // generated blends must then land on the anchor-interpolated turn rate.
    {
        pmg::ParametricMotionSpace turning("turning", 1);
        turning.AddExample({0.0f}, MakeTurningClip(-30.0f, 31));
        turning.AddExample({1.0f}, MakeTurningClip(30.0f, 31));

        const pmg::ParameterCalibration table =
            pmg::CalibrateParameterMetric(
                turning, pmg::ParameterMetric::kTurnRate, 30.0f, 5);
        assert(table.metrics.size() == 1);
        assert(table.samples.size() == 7);
        assert(table.example_measured.front().front() < 0.0f);
        assert(table.example_measured.back().front() > 0.0f);
        turning.SetParameterCalibration(table);

        for (const float parameter : {0.25f, 0.5f, 0.75f}) {
            const float target_rate =
                table.example_measured.front().front() +
                parameter * (table.example_measured.back().front() -
                             table.example_measured.front().front());
            const float achieved_rate = pmg::MeasureParameterMetric(
                pmg::ParameterMetric::kTurnRate,
                turning.GenerateClip({parameter}, 30.0f));
            assert(std::abs(achieved_rate - target_rate) < 0.05f);
        }
    }

    // Multidimensional calibration: turn rate and travel speed are measured
    // together, normalized by their sampled ranges, and inverted to full
    // example weights deterministically.
    {
        pmg::ParametricMotionSpace locomotion("locomotion", 2);
        locomotion.AddExample(
            {-1.0f, 1.0f}, MakeLocomotionClip(-45.0f, 1.0f, 31));
        locomotion.AddExample(
            {1.0f, 1.0f}, MakeLocomotionClip(45.0f, 1.0f, 31));
        locomotion.AddExample(
            {-1.0f, 3.0f}, MakeLocomotionClip(-45.0f, 3.0f, 31));
        locomotion.AddExample(
            {1.0f, 3.0f}, MakeLocomotionClip(45.0f, 3.0f, 31));

        const std::vector<pmg::ParameterMetric> metrics = {
            pmg::ParameterMetric::kTurnRate,
            pmg::ParameterMetric::kTravelSpeed,
        };
        const pmg::ParameterCalibration first =
            pmg::CalibrateParameterMetrics(locomotion, metrics, 30.0f, 5);
        const pmg::ParameterCalibration second =
            pmg::CalibrateParameterMetrics(locomotion, metrics, 30.0f, 5);
        assert(first.metrics == metrics);
        assert(first.metric_scales.size() == 2);
        assert(first.samples.size() == 29);
        assert(first.samples_per_axis == 5);
        assert(second.samples.size() == first.samples.size());
        for (std::size_t sample = 0; sample < first.samples.size(); ++sample) {
            assert(first.samples[sample].measured_parameter ==
                   second.samples[sample].measured_parameter);
            assert(first.samples[sample].blend_weights ==
                   second.samples[sample].blend_weights);
        }

        const pmg::ParameterVector query{0.35f, 2.2f};
        const std::vector<float> authored_weights =
            locomotion.ComputeLocalBlendWeights(query);
        pmg::ParameterVector target_measured(2, 0.0f);
        for (std::size_t example = 0;
             example < authored_weights.size(); ++example) {
            for (std::size_t metric = 0; metric < metrics.size(); ++metric) {
                target_measured[metric] +=
                    authored_weights[example] *
                    first.example_measured[example][metric];
            }
        }

        locomotion.SetParameterCalibration(first);
        const pmg::MotionClip generated =
            locomotion.GenerateClip(query, 30.0f);
        const float achieved_turn_rate = pmg::MeasureParameterMetric(
            pmg::ParameterMetric::kTurnRate, generated);
        const float achieved_travel_speed = pmg::MeasureParameterMetric(
            pmg::ParameterMetric::kTravelSpeed, generated);
        assert(std::abs(achieved_turn_rate - target_measured[0]) < 0.15f);
        assert(std::abs(achieved_travel_speed - target_measured[1]) < 0.2f);
    }

    return 0;
}
