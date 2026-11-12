# Highlights

Not part of the manuscript (the target journal does not ask for highlights); kept for a
submission system or a summary page that does. No number is typed here: the figures are in
the abstract and in Section 9, which take them from `generated/numbers.tex`.

- A battery of 33 conforming-contact problems with analytic references, classified by what each
  reference assumes; a subset was written before any model was run on it.
- Two elastic-foundation laws on exact mesh intersections give the same errors on Drake, Chrono
  and MuJoCo, which checks their implementation.
- With capped errors, failures counted and a multiplicity correction, the distributed law is
  more accurate than Gonthier's and than Chrono's penalty contact where the pressure
  distribution matters; on rigid-body references no difference from the native models is
  significant.
- A closed-form Coulomb friction wrench per triangle makes the friction of a flat face
  independent of its triangulation; element quadratures need a refined, well-shaped mesh.
- The laws fail by their explicit coupling, by the rolling resistance of the damped foundation,
  by one normal per convex pair and by a friction-induced flutter on stiffer foundations, whose
  stability condition is given.
