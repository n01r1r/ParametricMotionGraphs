# Phase 2 Notes

Phase 2 should be triggered by observed failure cases in Phase 1, not by speculative novelty.

## Candidate Extensions

| Observed Phase-1 Problem | Extension |
|---|---|
| Some source parameters can transition but edge is rejected globally | Partial-edge source domains |
| User input reacts too late | Phase-aware / mid-clip transitions |
| Foot sliding or contact artifacts dominate | Contact-aware transition metric |
| Parameter-near motions are not actually close | Manifold or bilateral parameterization |
| Valid target set is non-box-like | Learned transition validity or non-AABB regions |
| Linear blend pops | Inertialized transition |

## Manifold Parameterization Axis

Compare manual parameterization with:

- PCA;
- Isomap;
- LLE;
- Laplacian eigenmaps;
- diffusion maps;
- autoencoder latent coordinates.

Evaluation should measure reconstruction error, interpolation smoothness, transition coverage, transition artifacts, and runtime cost.

## Bilateral / Multi-Domain Transition Axis

A transition may be modeled in a joint domain:

```text
x = [motion_parameter, phase, contact_state, root_velocity, control_goal]
```

Then transition validity is:

```text
V(x_source, x_target)
```

This should be added only after the original PMG pipeline produces reproducible failure cases.
