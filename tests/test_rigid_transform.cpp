#include "pmg/RigidTransform2D.h"

#include <cassert>
#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;

bool Near(float a, float b, float eps = 1.0e-5f) {
    return std::abs(a - b) < eps;
}

}  // namespace

int main() {
    // Identity leaves a floor point and a pose untouched.
    {
        const pmg::RigidTransform2D id = pmg::RigidTransform2D::Identity();
        float x = 0.0f, z = 0.0f;
        id.ApplyFloor(3.0f, -4.0f, x, z);
        assert(Near(x, 3.0f) && Near(z, -4.0f));

        pmg::Pose pose;
        pose.root_position = {1.0f, 2.0f, 3.0f};
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        const pmg::Pose world = id.Apply(pose);
        assert(Near(world.root_position.x, 1.0f));
        assert(Near(world.root_position.y, 2.0f));  // y never touched
        assert(Near(world.root_position.z, 3.0f));
    }

    // RotateFloor convention: x' = cos*x + sin*z, z' = -sin*x + cos*z.
    // yaw = +90deg maps (1,0) -> (0,-1).
    {
        pmg::RigidTransform2D t;
        t.yaw = kPi / 2.0f;
        float x = 0.0f, z = 0.0f;
        t.RotateFloor(1.0f, 0.0f, x, z);
        assert(Near(x, 0.0f) && Near(z, -1.0f));
    }

    // ApplyFloor = rotate then translate.
    {
        pmg::RigidTransform2D t;
        t.yaw = kPi / 2.0f;
        t.dx = 10.0f;
        t.dz = -5.0f;
        float x = 0.0f, z = 0.0f;
        t.ApplyFloor(1.0f, 0.0f, x, z);
        assert(Near(x, 10.0f) && Near(z, -6.0f));
    }

    // Compose matches sequential application: (outer o inner)(p) == outer(inner(p)).
    {
        pmg::RigidTransform2D inner;
        inner.yaw = 0.6f; inner.dx = 2.0f; inner.dz = -1.0f;
        pmg::RigidTransform2D outer;
        outer.yaw = -1.1f; outer.dx = -3.0f; outer.dz = 4.0f;

        const pmg::RigidTransform2D composed =
            pmg::RigidTransform2D::Compose(outer, inner);

        const float px = 1.7f, pz = -2.3f;
        float ix = 0.0f, iz = 0.0f;
        inner.ApplyFloor(px, pz, ix, iz);
        float seq_x = 0.0f, seq_z = 0.0f;
        outer.ApplyFloor(ix, iz, seq_x, seq_z);

        float comp_x = 0.0f, comp_z = 0.0f;
        composed.ApplyFloor(px, pz, comp_x, comp_z);

        assert(Near(comp_x, seq_x) && Near(comp_z, seq_z));
        assert(Near(composed.yaw, outer.yaw + inner.yaw));
    }

    // Identity is a composition unit on both sides.
    {
        pmg::RigidTransform2D t;
        t.yaw = 0.3f; t.dx = 5.0f; t.dz = 7.0f;
        const pmg::RigidTransform2D id = pmg::RigidTransform2D::Identity();
        const pmg::RigidTransform2D left = pmg::RigidTransform2D::Compose(id, t);
        const pmg::RigidTransform2D right = pmg::RigidTransform2D::Compose(t, id);
        assert(Near(left.yaw, t.yaw) && Near(left.dx, t.dx) && Near(left.dz, t.dz));
        assert(Near(right.yaw, t.yaw) && Near(right.dx, t.dx) && Near(right.dz, t.dz));
    }

    // Inverse undoes Apply and composes to Identity on both sides. This backs
    // the backward cycle fold (negative pre-roll) in RuntimeController.
    {
        pmg::RigidTransform2D t;
        t.yaw = -0.8f; t.dx = 4.0f; t.dz = -2.5f;
        const pmg::RigidTransform2D inv = t.Inverse();

        // inv(t(p)) == p for a sample floor point.
        const float px = 1.3f, pz = 0.4f;
        float fx = 0.0f, fz = 0.0f;
        t.ApplyFloor(px, pz, fx, fz);
        float bx = 0.0f, bz = 0.0f;
        inv.ApplyFloor(fx, fz, bx, bz);
        assert(Near(bx, px) && Near(bz, pz));

        // Compose(inv, t) and Compose(t, inv) are both Identity.
        const pmg::RigidTransform2D left = pmg::RigidTransform2D::Compose(inv, t);
        const pmg::RigidTransform2D right = pmg::RigidTransform2D::Compose(t, inv);
        assert(Near(left.yaw, 0.0f) && Near(left.dx, 0.0f) && Near(left.dz, 0.0f));
        assert(Near(right.yaw, 0.0f) && Near(right.dx, 0.0f) && Near(right.dz, 0.0f));
    }

    return 0;
}
