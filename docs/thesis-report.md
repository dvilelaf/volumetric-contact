# Origin: the 2018 thesis

This repository grew out of the doctoral thesis

> Vilela Freire, D. *Simulation of conforming contact in real-time multibody dynamics using a
> volumetric force model.* PhD thesis, Universidade da Coruña, Laboratorio de Ingeniería Mecánica,
> 2018 (defended 10 December 2018; advisors Manuel González Castro and Alberto Luaces).
> Open access in the university repository: <https://hdl.handle.net/2183/21530> (RUC).

## What the thesis did

- **Formulation.** The multibody dynamics used the index-3 augmented Lagrangian formulation (ALI3-P)
  in natural coordinates, integrated with the trapezoidal rule at a fixed 1 ms step, in the
  laboratory's Fortran code MBSLIM.
- **Contact model.** Gonthier's volumetric model: the normal force is proportional to the volume of
  the intersection of the two bodies, applied at its centroid, with rolling resistance and a
  stiction (bristle) friction with spinning friction.
- **Two ways to compute the intersection volume.** (1) Triangle meshes: an AABB tree finds the
  candidate triangles, each is clipped against the other body's solid BSP tree, and the exact
  volume, centroid and inertia follow from the clipped polyhedron; the contact normal is a
  least-squares plane fit to the contact contour. (2) Inner sphere trees (bodies filled with
  thousands of spheres, sphere–sphere contacts summed).
- **Tests.** A sliding block, a spinning disk and a rolling cylinder. The mesh method was 6 to 19
  times faster than the sphere trees and ran in real time in two of the three tests (Intel Core i7 950).

## The limitation it stated

The mesh method gives one force with one normal per pair of bodies, so it only works when the
contact contour is approximately planar: it cannot represent, for instance, a cube wedged in a hole.
Its friction acts at a single point, the centroid of the overlap, which also limits how pressure and
slip distribute over a large contact patch.

## How this repository relates to it

This is a new implementation, written in 2026 in C++20, not a continuation of the thesis code, which
is not public. It keeps the thesis's idea (the exact mesh intersection as the basis of the contact
force), reimplements Gonthier's law as published and in the form the thesis used it
(`gonthier` and `gonthier-thesis`, see `gonthier-model.md`), and removes the single-force limitation
with a second law that integrates the same foundation over the contact surface (`distributed`). It
is independent of the multibody engine, and validates every law against analytic solutions and
against the native contact of Drake, Project Chrono and MuJoCo. The history of decisions is in
`history.md`; the scenarios, in `scenarios.md`.

The three tests of the thesis (block, disk, cylinder) are the first scenarios of the battery. Their
meshes were procedural from the start of this repository, built to the same criteria as the others
(`../data/README.md`).
