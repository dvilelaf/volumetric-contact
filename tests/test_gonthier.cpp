#include <cmath>
#include <numbers>

#include <doctest/doctest.h>

#include "test_util.hpp"
#include "volcontact/contact_model.hpp"
#include "volcontact/gonthier.hpp"

using namespace volcontact;
using volcontact::test::make_box;

namespace {

IntersectionProperties slab(double volume) {
  IntersectionProperties g;
  g.volume = volume;
  g.centroid = Vec3(0, 0, -0.01);
  g.inertia = Vec3(1e-3, 1e-3, 2e-3).asDiagonal();
  g.normal = Vec3::UnitZ();
  return g;
}

// Overlap of a flat disk of the given radius pressed `depth` into a plane: r_gyr = R/√2.
IntersectionProperties disk_overlap(double radius, double depth) {
  IntersectionProperties g;
  g.volume = std::numbers::pi * radius * radius * depth;
  g.centroid = Vec3(0, 0, -0.5 * depth);
  const double across = radius * radius / 4.0 + depth * depth / 12.0;
  g.inertia = (g.volume * Vec3(across, across, radius * radius / 2.0)).asDiagonal();
  g.normal = Vec3::UnitZ();
  return g;
}

Pose turned(double angle) { return {Eigen::AngleAxisd(angle, Vec3::UnitZ()).toRotationMatrix(), Vec3::Zero()}; }

constexpr auto kThesis = GonthierParams::FrictionModel::kThesis2018;

}  // namespace

TEST_CASE("normal force is K·V with Hunt-Crossley-like damping") {
  const GonthierParams p{.stiffness = 1e6, .damping = 5.0, .mu_spin = 0.5};
  const Pose X_WA{Mat3::Identity(), Vec3(0, 0, 0.24)};
  FrictionState next;

  const ContactForce rest = gonthier_force(slab(1e-5), X_WA, Twist{}, Pose{}, Twist{}, p, {}, &next);
  CHECK(rest.normal_force == doctest::Approx(10.0));
  CHECK(rest.force.isApprox(Vec3(0, 0, 10.0)));

  const Twist approaching{Vec3::Zero(), Vec3(0, 0, -0.1)};
  const ContactForce moving = gonthier_force(slab(1e-5), X_WA, approaching, Pose{}, Twist{}, p, {}, &next);
  CHECK(moving.normal_force == doctest::Approx(10.0 * 1.5));
}

TEST_CASE("normal force is clamped at zero unless adhesion is allowed") {
  GonthierParams p{.stiffness = 1e6, .damping = 5.0, .mu_spin = 0.5};
  const Twist separating{Vec3::Zero(), Vec3(0, 0, 0.3)};  // Faster than 1/h.
  FrictionState next;
  CHECK(gonthier_force(slab(1e-5), Pose{}, separating, Pose{}, Twist{}, p, {}, &next).normal_force == 0.0);
  p.allow_adhesion = true;
  CHECK(gonthier_force(slab(1e-5), Pose{}, separating, Pose{}, Twist{}, p, {}, &next).normal_force ==
        doctest::Approx(10.0 * (1.0 - 5.0 * 0.3)));
}

TEST_CASE("thesis 2018 variant: viscous spinning torque, sliding force independent of the spin") {
  GonthierParams p{.stiffness = 1e6, .damping = 5.0, .mu_spin = 0.6, .friction_model = kThesis};
  FrictionState next;
  const Twist spin{Vec3(0.3, 0, 2.0), Vec3::Zero()};
  const ContactForce f = gonthier_force(slab(1e-5), Pose{}, spin, Pose{}, Twist{}, p, {}, &next);
  CHECK(f.torque.z() == doctest::Approx(-0.6 * 10.0 / 1e-5 * 2e-3 * 2.0));
  CHECK(f.torque.x() == doctest::Approx(-1e6 * 5.0 * 1e-3 * 0.3));

  // With friction, a spinning and sliding contact gets the whole μ·Fn against the slide.
  p.damping = 0.0;
  p.friction = BristleFriction{.mu_dynamic = 0.5, .mu_static = 0.5};
  const Twist both{Vec3(0, 0, 2.0), Vec3(1.0, 0, 0)};
  const ContactForce g = gonthier_force(slab(1e-5), Pose{}, both, Pose{}, Twist{}, p, {}, &next);
  CHECK(g.force.x() == doctest::Approx(-5.0));
  CHECK(g.torque.z() == doctest::Approx(-0.6 * 10.0 / 1e-5 * 2e-3 * 2.0));
  CHECK(next.twist == 0.0);
}

TEST_CASE("rolling resistance opposes the tangential angular velocity, and vanishes with a clamped normal force") {
  GonthierParams p{.stiffness = 1e6, .damping = 5.0};
  FrictionState next;
  const Twist rolling{Vec3(0.3, 0, 0), Vec3::Zero()};
  CHECK(gonthier_force(slab(1e-5), Pose{}, rolling, Pose{}, Twist{}, p, {}, &next).torque.x() ==
        doctest::Approx(-1e6 * 5.0 * 1e-3 * 0.3));

  // Separating faster than 1/h: no pressure, so no moment of the pressure either.
  const Twist leaving{Vec3(0.3, 0, 0), Vec3(0, 0, 0.3)};
  const ContactForce clamped = gonthier_force(slab(1e-5), Pose{}, leaving, Pose{}, Twist{}, p, {}, &next);
  CHECK(clamped.normal_force == 0.0);
  CHECK(clamped.torque.isZero());
  p.allow_adhesion = true;
  CHECK(gonthier_force(slab(1e-5), Pose{}, leaving, Pose{}, Twist{}, p, {}, &next).torque.x() ==
        doctest::Approx(-1e6 * 5.0 * 1e-3 * 0.3));
}

TEST_CASE("published: pure spin gets the Coulomb torque μ·Fn·r_gyr whatever the speed, and no force") {
  const double R = 0.1, fn = 9.81, mu = 0.5;
  GonthierParams p{.stiffness = 1e6, .damping = 0.0};
  p.friction = BristleFriction{.mu_dynamic = mu, .mu_static = mu};
  for (const double w : {2.0, -20.0, 200.0}) {
    FrictionState next;
    const ContactForce f =
        gonthier_force(disk_overlap(R, fn / (1e6 * std::numbers::pi * R * R)), Pose{}, Twist{Vec3(0, 0, w), Vec3::Zero()},
                       Pose{}, Twist{}, p, {}, &next);
    CHECK(f.torque.z() == doctest::Approx(-std::copysign(mu * fn * R / std::numbers::sqrt2, w)).epsilon(1e-9));
    CHECK(f.torque.x() == 0.0);
    CHECK(f.torque.y() == 0.0);
    CHECK(f.force.x() == 0.0);
    CHECK(f.force.y() == 0.0);
  }
}

TEST_CASE("published: a spinning disk decelerates linearly, stops at I·ω0/(μ·Fn·r_gyr) and stays") {
  const double dt = 1e-3, R = 0.1, mass = 1.0, mu = 0.5, w0 = 10.0;
  const double fn = mass * 9.81, inertia = 0.5 * mass * R * R;
  const double t_stop = inertia * w0 / (mu * fn * R / std::numbers::sqrt2);  // 0.144 s.
  GonthierParams p{.stiffness = 1e6, .damping = 0.0};
  p.friction = BristleFriction{.mu_dynamic = mu, .mu_static = mu};
  const IntersectionProperties geo = disk_overlap(R, fn / (1e6 * std::numbers::pi * R * R));
  double angle = 0.0, w = w0, w_half = 0.0, t_slow = -1.0, angle_mid = 0.0;
  FrictionState state;
  for (int i = 0; i < 600; ++i) {
    FrictionState next;
    const ContactForce f =
        gonthier_force(geo, turned(angle), Twist{Vec3(0, 0, w), Vec3::Zero()}, Pose{}, Twist{}, p, state, &next);
    state = next;
    angle += dt * w;
    w += dt * f.torque.z() / inertia;
    if (i + 1 == 72) w_half = w;
    if (t_slow < 0.0 && w < 0.1 * w0) t_slow = (i + 1) * dt;
    if (i + 1 == 400) angle_mid = angle;
  }
  CHECK(w_half == doctest::Approx(w0 * (1.0 - 0.072 / t_stop)).epsilon(0.01));
  CHECK(t_slow == doctest::Approx(0.9 * t_stop).epsilon(0.02));
  CHECK(std::abs(w) < 1e-6);
  CHECK(angle == doctest::Approx(angle_mid).epsilon(1e-6));  // Held by the torsional bristle.
}

TEST_CASE("published: sliding and spinning share the friction budget through v_avg") {
  const double R = 0.1, fn = 9.81, mu = 0.5, r_gyr = R / std::numbers::sqrt2;
  GonthierParams p{.stiffness = 1e6, .damping = 0.0};
  p.friction = BristleFriction{.mu_dynamic = mu, .mu_static = mu};
  const IntersectionProperties geo = disk_overlap(R, fn / (1e6 * std::numbers::pi * R * R));
  for (const double w : {0.5, 5.0, 50.0}) {
    const Vec3 v(0.3, 0.4, 0);
    FrictionState next;
    const ContactForce f = gonthier_force(geo, Pose{}, Twist{Vec3(0, 0, w), v}, Pose{}, Twist{}, p, {}, &next);
    const double v_avg = std::sqrt(v.squaredNorm() + r_gyr * r_gyr * w * w);
    const Vec3 tangential(f.force.x(), f.force.y(), 0);
    CHECK((tangential + mu * fn * v / v_avg).norm() < 1e-9);
    CHECK(f.torque.z() == doctest::Approx(-mu * fn * r_gyr * r_gyr * w / v_avg).epsilon(1e-9));
    const double force_share = tangential.norm() / (mu * fn), torque_share = std::abs(f.torque.z()) / (mu * fn * r_gyr);
    CHECK(force_share * force_share + torque_share * torque_share == doctest::Approx(1.0).epsilon(1e-9));
  }
}

TEST_CASE("published, with the anchored bristle: pure sliding is the same as in the thesis variant") {
  GonthierParams published{.stiffness = 1e6, .damping = 0.0};
  published.friction = BristleFriction{.deflection = BristleFriction::Deflection::kAnchored};
  GonthierParams thesis = published;
  thesis.friction_model = kThesis;
  // Stick, transition and slip, back and forth, both above and below v_ε = v_stick/100.
  FrictionState sp, st;
  double x = 0.0;
  for (const double v : {1e-3, 1e-2, 0.1, 1.0, 0.02, -3e-4, -0.5, 1e-5, 1e-4, 0.0, 2e-3}) {
    x += 2e-5 * (v > 0.0 ? 1.0 : -1.0);
    const Pose X{Mat3::Identity(), Vec3(x, 0, 0)};
    const Twist V{Vec3::Zero(), Vec3(v, 0, 0)};
    FrictionState np, nt;
    const ContactForce fp = gonthier_force(slab(1e-5), X, V, Pose{}, Twist{}, published, sp, &np);
    const ContactForce ft = gonthier_force(slab(1e-5), X, V, Pose{}, Twist{}, thesis, st, &nt);
    sp = np;
    st = nt;
    CAPTURE(v);
    if (std::abs(v) >= 0.01 * published.friction->v_stick || v == 0.0) {
      CHECK(fp.force == ft.force);
      CHECK(np.anchor_B == nt.anchor_B);
    } else {  // Below v_ε the published C_v is regularized: at most 1.5e-5 of the limit.
      CHECK((fp.force - ft.force).norm() < 2e-5 * 5.0);
    }
    CHECK(fp.torque.isZero());
  }
}

TEST_CASE("published: torsional stiction holds a torque below μ·Fn·r_gyr without creep") {
  const double dt = 1e-3, R = 0.1, mu = 0.5, r_gyr = R / std::numbers::sqrt2;
  for (const double mass : {1.0, 100.0}) {
    const double fn = mass * 9.81, inertia = 0.5 * mass * R * R;
    GonthierParams p{.stiffness = 1e6, .damping = 0.0};
    p.friction = BristleFriction{.mu_dynamic = mu, .mu_static = mu};
    const IntersectionProperties geo = disk_overlap(R, fn / (1e6 * std::numbers::pi * R * R));
    const double applied = 0.5 * mu * fn * r_gyr;
    double angle = 0.0, w = 0.0, angle_mid = 0.0;
    FrictionState state;
    for (int i = 0; i < 5000; ++i) {
      FrictionState next;
      const ContactForce f =
          gonthier_force(geo, turned(angle), Twist{Vec3(0, 0, w), Vec3::Zero()}, Pose{}, Twist{}, p, state, &next);
      state = next;
      angle += dt * w;
      w += dt * (applied + f.torque.z()) / inertia;
      if (i == 2499) angle_mid = angle;
    }
    CHECK(angle == doctest::Approx(0.5 * p.friction->preslide_travel / r_gyr).epsilon(1e-3));
    CHECK(angle == doctest::Approx(angle_mid).epsilon(1e-4));
    CHECK(std::abs(w) < 1e-6);
  }
}

TEST_CASE("a bristle loaded at exactly zero slip speed keeps its force") {
  for (const auto model : {GonthierParams::FrictionModel::kPublished, kThesis}) {
    GonthierParams p{.stiffness = 1e6, .damping = 0.0, .friction_model = model};
    p.friction = BristleFriction{.mu_dynamic = 0.5, .mu_static = 0.5, .k_stick = 400, .c_stick = 0,
                                .preslide_travel = 0.0};
    p.friction_point = {.kind = FrictionPoint::Kind::kBodyPoint, .p_A = Vec3(0, 0, -0.01)};
    FrictionState first, after;
    gonthier_force(slab(1e-5), Pose{}, Twist{}, Pose{}, Twist{}, p, {}, &first);
    REQUIRE(first.active);

    // 1 mm from the anchor, at rest: k·d = 0.4 N.
    const Pose moved{Mat3::Identity(), Vec3(1e-3, 0, 0)};
    CHECK(gonthier_force(slab(1e-5), moved, Twist{}, Pose{}, Twist{}, p, first, &after).force.x() ==
          doctest::Approx(-0.4));
    const bool anchored = model == kThesis;
    if (anchored) CHECK(after.anchor_B.isApprox(first.anchor_B));
    else CHECK(after.deflection_B.isApprox(Vec3(1e-3, 0, 0)));

    // 10 cm, at rest: the spring saturates at μ·Fn = 5 N and the anchor follows along the bristle.
    const Pose far{Mat3::Identity(), Vec3(0.1, 0, 0)};
    CHECK(gonthier_force(slab(1e-5), far, Twist{}, Pose{}, Twist{}, p, first, &after).force.x() ==
          doctest::Approx(-5.0));
    if (anchored) CHECK(after.anchor_B.x() == doctest::Approx(0.1 - 5.0 / 400.0));
    else CHECK(after.deflection_B.x() == doctest::Approx(5.0 / 400.0));
  }
}

TEST_CASE("bristle friction sticks below the static limit and slides above it") {
  GonthierParams p{.stiffness = 1e6, .damping = 0.0, .mu_spin = 0.5};
  p.friction = BristleFriction{.mu_dynamic = 0.5, .mu_static = 0.5, .v_stick = 0.01, .k_stick = 400, .c_stick = 0,
                              .preslide_travel = 0.0};  // Constant mode: the thesis' numbers.
  p.friction_point = {.kind = FrictionPoint::Kind::kBodyPoint, .p_A = Vec3(0, 0, -0.01)};
  FrictionState next;

  // The bristle of a new contact is unloaded.
  const Twist creep{Vec3::Zero(), Vec3(1e-5, 0, 0)};
  gonthier_force(slab(1e-5), Pose{}, creep, Pose{}, Twist{}, p, {}, &next);
  REQUIRE(next.active);
  CHECK(next.deflection_B.isZero());

  // Displaced 1 mm at near-zero speed: spring force k·d = 0.4 N.
  const Pose moved{Mat3::Identity(), Vec3(1e-3, 0, 0)};
  FrictionState after;
  const ContactForce stick = gonthier_force(slab(1e-5), moved, creep, Pose{}, Twist{}, p, next, &after);
  CHECK(stick.force.x() == doctest::Approx(-0.4).epsilon(1e-3));
  CHECK(after.deflection_B.isApprox(Vec3(1e-3, 0, 0)));

  // Displaced 10 cm: the spring saturates at μ·Fn = 5 N and the bristle slips.
  const Pose far{Mat3::Identity(), Vec3(0.1, 0, 0)};
  const ContactForce slip = gonthier_force(slab(1e-5), far, creep, Pose{}, Twist{}, p, next, &after);
  CHECK(slip.force.x() == doctest::Approx(-5.0).epsilon(1e-3));
  CHECK(after.deflection_B.x() == doctest::Approx(5.0 / 400.0));  // The spring stays at the limit.
}

TEST_CASE("published: the bristle is loaded by the slip of the body, not by the drift of the overlap's centroid") {
  // The overlap of a block that rocks on a flat contact moves its centroid with nothing sliding.
  GonthierParams p{.stiffness = 1e6, .damping = 0.0};
  p.friction = BristleFriction{};
  const double d = p.friction->preslide_travel;
  IntersectionProperties shifted = slab(1e-5);
  shifted.centroid.x() += 5e-3;
  FrictionState first, after;
  gonthier_force(slab(1e-5), Pose{}, Twist{}, Pose{}, Twist{}, p, {}, &first);
  CHECK(gonthier_force(shifted, Pose{}, Twist{}, Pose{}, Twist{}, p, first, &after).force.x() == 0.0);
  CHECK(after.deflection_B.isZero());
  // And a body that slips 0.3·d under an overlap that stays where it was gets 0.3 of the limit.
  const Pose slipped{Mat3::Identity(), Vec3(0.3 * d, 0, 0)};
  CHECK(gonthier_force(slab(1e-5), slipped, Twist{}, Pose{}, Twist{}, p, first, &after).force.x() ==
        doctest::Approx(-0.3 * 5.0));

  // The anchored bristle of the 2018 thesis takes the drift for slip: it saturates.
  p.friction->deflection = BristleFriction::Deflection::kAnchored;
  gonthier_force(slab(1e-5), Pose{}, Twist{}, Pose{}, Twist{}, p, {}, &first);
  CHECK(gonthier_force(shifted, Pose{}, Twist{}, Pose{}, Twist{}, p, first, &after).force.x() ==
        doctest::Approx(-5.0));
  CHECK(gonthier_force(slab(1e-5), slipped, Twist{}, Pose{}, Twist{}, p, first, &after).force.x() == 0.0);
}

TEST_CASE("pre-sliding travel to the static limit does not depend on the load") {
  const double d = BristleFriction{}.preslide_travel;
  for (const double load : {1.0, 1000.0}) {
    GonthierParams p{.stiffness = 1e6, .damping = 0.0, .mu_spin = 0.5};
    p.friction = BristleFriction{.v_stick = 0.01};
    p.friction_point = {.kind = FrictionPoint::Kind::kBodyPoint, .p_A = Vec3(0, 0, -0.01)};
    const double volume = load / 1e6, limit = 0.5 * load;
    const Twist creep{Vec3::Zero(), Vec3(1e-9, 0, 0)};
    FrictionState first, after;
    gonthier_force(slab(volume), Pose{}, creep, Pose{}, Twist{}, p, {}, &first);
    const auto at = [&](double x) {
      const Pose X{Mat3::Identity(), Vec3(x, 0, 0)};
      return gonthier_force(slab(volume), X, creep, Pose{}, Twist{}, p, first, &after);
    };
    CHECK(at(0.3 * d).force.x() == doctest::Approx(-0.3 * limit).epsilon(1e-6));
    CHECK(at(0.01).force.x() == doctest::Approx(-limit).epsilon(1e-6));  // Slips at the limit.
    CHECK(after.deflection_B.x() == doctest::Approx(d));
  }
}

TEST_CASE("bodies of 1 kg and 100 kg held by stiction are stable with explicit Euler at 1 ms") {
  const double dt = 1e-3;
  for (const double mass : {1.0, 100.0}) {
    GonthierParams p{.stiffness = 1e6, .damping = 0.0, .mu_spin = 0.5};
    p.friction = BristleFriction{};
    p.friction_point = {.kind = FrictionPoint::Kind::kBodyPoint, .p_A = Vec3(0, 0, -0.01)};
    const double d = p.friction->preslide_travel, weight = mass * 9.81;
    const double pull = 0.5 * 0.5 * weight;  // Half the static limit.
    double x = 0.0, v = 0.0, x_mid = 0.0, peak = 0.0;
    FrictionState state;
    for (int i = 0; i < 5000; ++i) {
      FrictionState next;
      const ContactForce f = gonthier_force(slab(weight / 1e6), Pose{Mat3::Identity(), Vec3(x, 0, 0)},
                                            Twist{Vec3::Zero(), Vec3(v, 0, 0)}, Pose{}, Twist{}, p, state, &next);
      state = next;
      x += dt * v;
      v += dt * (pull + f.force.x()) / mass;
      peak = std::max(peak, std::abs(x));
      if (i == 2499) x_mid = x;
    }
    CHECK(x == doctest::Approx(0.5 * d).epsilon(1e-3));  // F/(μN)·d.
    CHECK(x == doctest::Approx(x_mid).epsilon(1e-4));    // No creep.
    CHECK(peak < 0.7 * d);
    CHECK(std::abs(v) < 1e-6);
  }
}

TEST_CASE("a block on an incline slides at g(sinθ − μcosθ) above the friction angle and holds below it") {
  const double dt = 1e-3, g = 9.81, mu = 0.5, mass = 1.0;
  for (const double deg : {30.0, 20.0}) {
    const double th = deg * std::numbers::pi / 180.0;
    GonthierParams p{.stiffness = 1e6, .damping = 0.0, .mu_spin = 0.5};
    p.friction = BristleFriction{.mu_dynamic = mu, .mu_static = mu};
    p.friction_point = {.kind = FrictionPoint::Kind::kBodyPoint, .p_A = Vec3(0, 0, -0.01)};
    const double pull = mass * g * std::sin(th), fn = mass * g * std::cos(th);
    double x = 0.0, v = 0.0, x_mid = 0.0;
    FrictionState state;
    for (int i = 0; i < 1000; ++i) {
      FrictionState next;
      const ContactForce f = gonthier_force(slab(fn / 1e6), Pose{Mat3::Identity(), Vec3(x, 0, 0)},
                                            Twist{Vec3::Zero(), Vec3(v, 0, 0)}, Pose{}, Twist{}, p, state, &next);
      state = next;
      x += dt * v;
      v += dt * (pull + f.force.x()) / mass;
      if (i == 499) x_mid = x;
    }
    if (deg > 26.6) {
      CHECK(v == doctest::Approx(g * (std::sin(th) - mu * std::cos(th))).epsilon(0.02));
    } else {
      CHECK(x < 2.0 * p.friction->preslide_travel);
      CHECK(x == doctest::Approx(x_mid).epsilon(1e-4));
    }
  }
}

TEST_CASE("contact model applies equal and opposite wrenches") {
  ContactModel model;
  const auto cube = std::make_shared<const CollisionShape>(make_box(Vec3::Constant(-0.5), Vec3::Constant(0.5)));
  const auto ground = std::make_shared<const CollisionShape>(make_box(Vec3(-5, -5, -1), Vec3(5, 5, 0)));
  model.add_pair({.body_a = 0, .body_b = 1, .shape_a = cube, .shape_b = ground, .law = GonthierParams{}});

  const std::vector<Pose> X{{Mat3::Identity(), Vec3(0.2, 0, 0.49)}, Pose{}};
  const std::vector<Twist> V(2);
  ContactModel::Evaluation eval;
  model.evaluate(X, V, model.initial_friction(), &eval);
  REQUIRE(eval.pairs[0].in_contact);
  CHECK(eval.wrenches[0].force.z() == doctest::Approx(1e6 * 0.01));
  CHECK((eval.wrenches[0].force + eval.wrenches[1].force).norm() == doctest::Approx(0.0).epsilon(1e-9));
  // The force acts below the cube's centre: no torque about it.
  CHECK(eval.wrenches[0].torque.norm() == doctest::Approx(0.0).epsilon(1e-9));
}
