#pragma once

#include <filesystem>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "volcontact/contact_model.hpp"
#include "volcontact/gonthier.hpp"
#include "volcontact/types.hpp"

namespace volcontact {

// The files of a mesh asset (docs/scenarios.md §1).
struct ShapeMeshes {
  std::filesystem::path surface;                // <name>.obj: the whole closed surface.
  std::vector<std::filesystem::path> pieces;    // <name>/*.obj, or {surface} for a convex asset.
  std::optional<std::filesystem::path> volume;  // volume.vtk, if present.
};

// Files of the asset `name` in `data_dir`/bodies/`name` or `data_dir`/grounds/`name`.
[[nodiscard]] ShapeMeshes shape_meshes(const std::filesystem::path& data_dir, const std::string& name);

// Analytic shape of a body, for engines whose native contact prefers primitives.
struct Primitive {
  enum class Kind { kBox, kCylinder, kSphere } kind = Kind::kBox;
  Vec3 size = Vec3::Ones();  // Box: side lengths. Cylinder: (radius, length, unused). Sphere: (radius, unused, unused).
  Pose X_BG;                 // Pose of the primitive in the body frame (cylinders along +z).
};

struct BodySpec {
  std::string name;
  // Mesh asset in the body frame, whose origin is the centre of mass.
  ShapeMeshes shape;
  double mass = 1.0;
  Mat3 inertia = Mat3::Identity();  // About the centre of mass, in the body frame.
  Pose X_W0;                        // Initial pose.
  Twist V_W0;                       // Initial velocity.
  // World force at the centre of mass as a function of time; empty if none.
  std::function<Vec3(double t)> applied_force;
  std::optional<Primitive> primitive;
};

// Static body. Its top face is the plane z = 0, which engines may use as a
// half-space for their native contact models.
struct GroundSpec {
  ShapeMeshes shape;
  bool planar = false;  // The top is the world plane z = 0 (native contact may use a half-space).
  Pose X_WG;
};

// A body taking part in a contact: one of the dynamic bodies or a ground.
struct BodyRef {
  enum class Kind { kBody, kGround } kind = Kind::kBody;
  int index = 0;

  static BodyRef body(int i) { return {Kind::kBody, i}; }
  static BodyRef ground(int i) { return {Kind::kGround, i}; }
};

// Volumetric contact between two bodies, with the parameters of each force law
// (they describe the force on `a`). The force law is chosen per run.
struct ContactSpec {
  BodyRef a;
  BodyRef b;
  double friction = 0.5;  // Coulomb μ of the pair, for native contact.
  GonthierParams gonthier;
  DistributedParams distributed;
};

// kPcm takes K, h and μ from `distributed`.
enum class ForceModel { kGonthier, kDistributed, kPcm };

// Physical target for an engine's own contact model: normal force K·V with
// damping (1 − h·vn); the friction of each pair is in its ContactSpec.
// Each backend translates it (e.g. Drake: hydroelastic modulus K with a 1 m slab).
struct NativeContactSpec {
  double stiffness = 1e6;  // K [N/m³].
  double damping = 5.0;    // h [s/m].
};

// Sampled motion of the dynamic bodies.
class Trajectory {
 public:
  explicit Trajectory(int num_bodies = 0) : num_bodies_(num_bodies) {}

  void reserve(std::size_t samples) {
    t_.reserve(samples);
    X_.reserve(samples * num_bodies_);
    V_.reserve(samples * num_bodies_);
  }
  void push(double t, std::span<const Pose> X, std::span<const Twist> V) {
    t_.push_back(t);
    X_.insert(X_.end(), X.begin(), X.begin() + num_bodies_);
    V_.insert(V_.end(), V.begin(), V.begin() + num_bodies_);
  }

  [[nodiscard]] std::size_t size() const { return t_.size(); }
  [[nodiscard]] int num_bodies() const { return num_bodies_; }
  [[nodiscard]] const std::vector<double>& times() const { return t_; }
  [[nodiscard]] const Pose& pose(std::size_t sample, int body) const { return X_[sample * num_bodies_ + body]; }
  [[nodiscard]] const Twist& twist(std::size_t sample, int body) const { return V_[sample * num_bodies_ + body]; }

 private:
  int num_bodies_;
  std::vector<double> t_;
  std::vector<Pose> X_;
  std::vector<Twist> V_;
};

struct Series {
  std::string name;
  std::vector<double> values;  // One per trajectory sample.
};

struct MetricReport {
  // Worst normalized deviation of every quantity the scenario's theory fixes
  // (dimensionless, >= 0); each one is also a scalar `err_<quantity>`. NaN if the
  // run did not produce one of them (e.g. a body that never stopped).
  double error = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::pair<std::string, double>> scalars;
  std::vector<Series> series;
};

// What a scenario's theory rests on (docs/scenarios.md §4).
enum class TheoryClass {
  kA,  // Rigid-body result: any contact model must converge to it as it stiffens.
  kB,  // Assumes a pressure distribution; the scalars give the band of admissible answers.
  kC,  // Winkler consistency: the theory contains K or integrates the law's own force.
};

[[nodiscard]] std::string_view to_string(TheoryClass c);

struct Scenario {
  std::string name;
  std::string description;
  TheoryClass theory_class = TheoryClass::kA;
  Vec3 gravity = Vec3(0.0, 0.0, -9.81);
  double duration = 1.0;
  std::vector<BodySpec> bodies;
  std::vector<GroundSpec> grounds;
  std::vector<ContactSpec> contacts;
  NativeContactSpec native;
  std::function<MetricReport(const Trajectory&)> metrics;
};

// The volumetric contact model numbers bodies as: dynamic bodies first, in
// order, then the grounds. Backends pass poses and twists in that order.
[[nodiscard]] int contact_body_index(const Scenario& scenario, BodyRef ref);
// One pair per combination of pieces of the two bodies of every contact. With `dt` > 0 the
// model is told that an engine integrates its forces explicitly with that step, on the
// scenario's bodies (ContactModel::set_explicit_coupling); every backend passes its step.
[[nodiscard]] ContactModel make_contact_model(const Scenario& scenario, ForceModel model, double dt = 0.0);

// Ground with the slab `slab_40000x40000x1000`, whose top is the plane z = 0.
[[nodiscard]] GroundSpec planar_ground(const std::filesystem::path& data_dir);

}  // namespace volcontact
