#include "pmg_test.h"
#include "../apps/viewer/SceneCoordinates.h"

#include <cassert>
#include <cmath>

int main() {
    for (float scale : {0.1f, 1.0f, 10.0f, 50.0f}) {
        pmgviewer::SceneCoordinates coordinates{scale, 12.0f, -3.0f};
        const pmg::Vec3 planted{2, 0, 4};
        const auto initial = coordinates.ToDisplay(planted);
        for (int frame = 0; frame < 60; ++frame) {
            // Changing torso/centroid position must not move a stationary foot.
            const pmg::Vec3 moving_root{static_cast<float>(frame), 2, 4};
            const auto foot = coordinates.ToDisplay(planted);
            const auto root = coordinates.ToDisplay(moving_root);
            assert((foot - initial).Norm() == 0);
            assert(std::abs((root.x - foot.x) - scale * (moving_root.x - planted.x)) < 1e-4f);
            assert((coordinates.ToNative(root) - moving_root).Norm() < 1e-4f);
        }
        assert((coordinates.ToNative(initial) - planted).Norm() < 1e-4f);
    }
}
