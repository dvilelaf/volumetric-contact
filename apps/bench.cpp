// Runs the scenarios on every available engine and contact method. Writes
// per-run time series (CSV) and a summary (JSON) to --out when given, one
// battery row per run (status, error, timing, variant) to --csv, and the terms
// of every error to --terms.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>

#include "volcontact/engine.hpp"
#include "volcontact/scenarios.hpp"

namespace {

namespace fs = std::filesystem;

struct Args {
  std::string engine = "all";
  std::string contact = "all";
  std::string scenario = "all";
  std::string variant;  // NAME[,NAME...] or all: mesh, dt, stiffness and perturbation from variants()
  fs::path csv;         // one battery row per run is appended here
  fs::path terms;       // one row per term of every error (long format) is appended here
  double time_limit = 0.0;  // wall seconds per run; 0 is none
  bool resume = false;      // skip the runs --csv already holds with status ok
  double dt = 1e-3;
  double max_duration = INFINITY;  // simulated seconds per scenario; a cut run only shows that it runs
  int repeat = 5;
  bool native_primitives = false;
  double native_resolution = 0.05;
  volcontact::Resolution mesh = volcontact::Resolution::kBase;
  double stiffness_scale = 1.0;
  int perturbation = 0;
  bool onset = false;
  bool list = false;
  bool list_native = false;
  fs::path data = "data";
  fs::path out;  // time series and summary per run; none when empty
  fs::path traj;  // viewer trajectories, <scenario>/<variant>/<engine>-<contact>.json; none when empty
};

[[noreturn]] void usage() {
  std::cerr << "usage: volcontact_bench [--engine NAME|all]\n"
               "                        [--contact METHOD[,METHOD...]|all]   METHOD: gonthier, gonthier-thesis,\n"
               "                                distributed, distributed-vertex, distributed-centroid, pcm,\n"
               "                                native, native:MODEL, native:all\n"
               "                        [--scenario NAME[,NAME...]|all] [--mesh coarse|base|fine] [--perturbed N]\n"
               "                        [--stiffness-scale X] [--variant NAME[,NAME...]|all]\n"
               "                        [--csv FILE] [--terms FILE] [--resume] [--time-limit S]\n"
               "                        [--dt S] [--max-duration S] [--repeat N]\n"
               "                        [--native-primitives] [--native-resolution M]\n"
               "                        [--onset] [--list] [--list-native]\n"
               "                        [--data DIR] [--out DIR] [--traj DIR]\n";
  std::exit(2);
}

Args parse(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> std::string {
      if (i + 1 >= argc) usage();
      return argv[++i];
    };
    if (arg == "--engine") a.engine = value();
    else if (arg == "--contact") a.contact = value();
    else if (arg == "--scenario") a.scenario = value();
    else if (arg == "--variant") a.variant = value();
    else if (arg == "--csv") a.csv = value();
    else if (arg == "--terms") a.terms = value();
    else if (arg == "--resume") a.resume = true;
    else if (arg == "--time-limit") a.time_limit = std::stod(value());
    else if (arg == "--stiffness-scale") a.stiffness_scale = std::stod(value());
    else if (arg == "--dt") a.dt = std::stod(value());
    else if (arg == "--max-duration") a.max_duration = std::stod(value());
    else if (arg == "--repeat") a.repeat = std::max(1, std::stoi(value()));
    else if (arg == "--native-primitives") a.native_primitives = true;
    else if (arg == "--native-resolution") a.native_resolution = std::stod(value());
    else if (arg == "--mesh") {
      const std::string v = value();
      if (v == "coarse") a.mesh = volcontact::Resolution::kCoarse;
      else if (v == "base") a.mesh = volcontact::Resolution::kBase;
      else if (v == "fine") a.mesh = volcontact::Resolution::kFine;
      else usage();
    } else if (arg == "--perturbed") a.perturbation = std::stoi(value());
    else if (arg == "--onset") a.onset = true;
    else if (arg == "--list") a.list = true;
    else if (arg == "--list-native") a.list_native = true;
    else if (arg == "--data") a.data = value();
    else if (arg == "--out") a.out = value();
    else if (arg == "--traj") a.traj = value();
    else usage();
  }
  if ((!a.csv.empty() || !a.terms.empty()) && a.variant.empty()) usage();
  if (a.resume && a.csv.empty()) usage();
  if (!a.csv.empty() && std::isfinite(a.max_duration)) usage();  // The error of a cut run is not a battery row.
  return a;
}

// One way of running the scenarios: a variant of variants(), or the flags.
struct Config {
  std::string name;  // empty without --variant
  volcontact::Resolution mesh;
  double dt;
  double stiffness_scale;
  int perturbation;
};

std::vector<Config> configs(const Args& a) {
  if (a.variant.empty()) return {{"", a.mesh, a.dt, a.stiffness_scale, a.perturbation}};
  std::vector<std::string> wanted;
  std::istringstream names(a.variant);
  for (std::string name; std::getline(names, name, ',');) wanted.push_back(name);
  std::vector<Config> list;
  for (const volcontact::Variant& v : volcontact::variants()) {
    if (a.variant == "all" || std::find(wanted.begin(), wanted.end(), v.name) != wanted.end()) {
      list.push_back({v.name, v.resolution, v.dt, v.stiffness_scale, v.perturbation});
    }
  }
  if (list.empty()) usage();
  return list;
}

// A contact method as the bench runs it: the engine's choice, plus whether the Gonthier law
// takes the thesis's friction (`gonthier-thesis`) and how the distributed law integrates its
// kinetic friction (`distributed-vertex`, `distributed-centroid`: the ablation).
struct Method {
  volcontact::ContactChoice choice;
  bool thesis = false;
  volcontact::FrictionQuadrature quadrature = volcontact::FrictionQuadrature::kClosedForm;
};

// The methods of a `--contact` argument (a comma-separated list) on `engine`.
std::vector<Method> methods(const volcontact::Engine& engine, const std::string& argument) {
  std::vector<Method> list;
  std::istringstream tokens(argument);
  for (std::string token; std::getline(tokens, token, ',');) {
    if (token == "gonthier-thesis") {
      list.push_back({{volcontact::ContactMethod::kGonthier, "", token}, true});
    } else if (token == "distributed-vertex") {
      list.push_back({{volcontact::ContactMethod::kDistributed, "", token}, false, volcontact::FrictionQuadrature::kVertex});
    } else if (token == "distributed-centroid") {
      list.push_back({{volcontact::ContactMethod::kDistributed, "", token}, false, volcontact::FrictionQuadrature::kCentroid});
    } else {
      for (const volcontact::ContactChoice& c : volcontact::contact_choices(engine, token)) list.push_back({c, false});
    }
  }
  return list;
}

// A scenario of the battery. One that fails to build keeps its error, so its
// runs report failed.
struct Item {
  std::string name;
  std::optional<volcontact::Scenario> scenario;
  std::string error;
};

// The scenarios of a configuration. A scenario not written yet is skipped, and so is one the
// variant does not apply to: a mesh series needs a curved asset, a stiffness series a theory
// that does not contain K (not class C).
std::vector<Item> make_items(const std::vector<std::string>& names, const Config& config, const fs::path& data_dir,
                             double max_duration) {
  const volcontact::ScenarioOptions options{
      .data_dir = data_dir, .resolution = config.mesh, .stiffness_scale = config.stiffness_scale};
  std::vector<Item> items;
  for (const std::string& name : names) {
    try {
      volcontact::Scenario s = volcontact::make_scenario(name, options);
      if (config.mesh != volcontact::Resolution::kBase && !volcontact::depends_on_resolution(name, data_dir)) continue;
      if (config.stiffness_scale != 1.0 && s.theory_class == volcontact::TheoryClass::kC) continue;
      volcontact::perturb(s, config.perturbation);
      s.duration = std::min(s.duration, max_duration);
      items.push_back({name, s, ""});
    } catch (const std::out_of_range&) {
      usage();
    } catch (const std::logic_error& e) {
      std::cerr << "skipping " << name << ": " << e.what() << "\n";
    } catch (const std::exception& e) {
      std::cerr << "failed to build " << name << ": " << e.what() << "\n";
      items.push_back({name, std::nullopt, e.what()});
    }
  }
  return items;
}

std::string json_number(double x) {
  if (!std::isfinite(x)) return "null";
  std::ostringstream os;
  os.precision(10);
  os << x;
  return os.str();
}

// Non-finite values are written as nan or inf, which the statistics read.
std::string csv_number(double x) {
  if (std::isnan(x)) return "nan";
  if (std::isinf(x)) return x > 0 ? "inf" : "-inf";
  std::ostringstream os;
  os.precision(10);
  os << x;
  return os.str();
}

// What a run left: its status and numbers, one row of the battery CSV.
struct Row {
  std::string_view engine, contact;
  std::string scenario, variant;
  std::string status;  // ok, nan (the run finished with a nan error), timeout or failed (it threw)
  double error = NAN, ratio = NAN, wall = NAN;
  std::int64_t steps = 0;
  double dt = 0.0;
  int segments = 0;
  double stiffness_scale = 1.0;
  std::string theory_class;
};

// The runs a battery CSV holds with status ok, as "scenario|engine|contact|variant".
std::set<std::string> done_runs(const fs::path& path) {
  std::set<std::string> done;
  std::ifstream in(path);
  std::string line;
  if (!std::getline(in, line)) return done;
  std::vector<std::string> header;
  for (std::istringstream h(line); std::getline(h, line, ',');) header.push_back(line);
  const auto column = [&](const std::string& name) {
    return static_cast<std::size_t>(std::find(header.begin(), header.end(), name) - header.begin());
  };
  const std::size_t engine = column("engine"), contact = column("contact"), scenario = column("scenario"),
                    variant = column("variant"), status = column("status");
  while (std::getline(in, line)) {
    std::vector<std::string> f;
    std::istringstream l(line);
    for (std::string x; std::getline(l, x, ',');) f.push_back(x);
    if (f.size() > std::max({engine, contact, scenario, variant, status}) && f[status] == "ok") {
      done.insert(f[scenario] + "|" + f[engine] + "|" + f[contact] + "|" + f[variant]);
    }
  }
  return done;
}

// Appends `header` first when the file is new.
std::ofstream open_append(const fs::path& path, const char* header) {
  const bool fresh = !fs::exists(path);
  if (path.has_parent_path()) fs::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::app);
  if (fresh) out << header << '\n';
  return out;
}

// A timeout or a failure has no error or ratio: the fields are empty.
void append_row(const fs::path& path, const Row& r) {
  std::ofstream out = open_append(
      path, "engine,contact,scenario,variant,status,error,realtime_ratio,wall_s,steps,dt,segments,stiffness_scale,class");
  const bool measured = r.status == "ok" || r.status == "nan";
  out << r.engine << ',' << r.contact << ',' << r.scenario << ',' << r.variant << ',' << r.status << ','
      << (measured ? csv_number(r.error) : "") << ',' << (measured ? csv_number(r.ratio) : "") << ','
      << csv_number(r.wall) << ',' << r.steps << ',' << csv_number(r.dt) << ',' << r.segments << ','
      << csv_number(r.stiffness_scale) << ',' << r.theory_class << '\n';
}

// The terms of the error (scalars `err_<name>`), one line each.
void append_terms(const fs::path& path, const Row& r, const std::vector<std::pair<std::string, double>>& scalars) {
  std::ofstream out = open_append(path, "scenario,engine,contact,variant,term,value");
  for (const auto& [name, value] : scalars) {
    if (name.starts_with("err_")) {
      out << r.scenario << ',' << r.engine << ',' << r.contact << ',' << r.variant << ',' << name.substr(4) << ','
          << csv_number(value) << '\n';
    }
  }
}

void write_csv(const fs::path& path, const volcontact::Trajectory& traj, const volcontact::MetricReport& report) {
  std::ofstream out(path);
  out.precision(12);
  out << "t";
  for (const auto& s : report.series) out << ',' << s.name;
  out << '\n';
  for (std::size_t i = 0; i < traj.size(); ++i) {
    out << traj.times()[i];
    for (const auto& s : report.series) out << ',' << s.values[i];
    out << '\n';
  }
}

// Six significant digits; null for nan/inf, which JSON lacks.
std::string short_number(double x) {
  if (!std::isfinite(x)) return "null";
  std::ostringstream os;
  os.precision(6);
  os << x;
  return os.str();
}

// Pose as [x,y,z,qw,qx,qy,qz].
std::string pose_json(const volcontact::Pose& X) {
  const Eigen::Quaterniond q(X.R);
  std::string s = "[";
  for (const double v : {X.p.x(), X.p.y(), X.p.z(), q.w(), q.x(), q.y(), q.z()}) s += short_number(v) + ",";
  s.back() = ']';
  return s;
}

std::string json_string(const std::string& x) {
  std::string r = "\"";
  for (const char c : x) {
    if (c == '"' || c == '\\') r += '\\';
    r += c;
  }
  return r + "\"";
}

// What the web viewer (tools/viewer) shows of one run: the surfaces of grounds
// and bodies, the poses at most every 10 ms, and the metric series at those samples.
void write_traj(const fs::path& path, const volcontact::Scenario& s, const volcontact::Trajectory& traj,
                const volcontact::MetricReport& report) {
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < traj.size(); ++i) {
    if (idx.empty() || traj.times()[i] >= traj.times()[idx.back()] + 0.01 - 1e-9) idx.push_back(i);
  }
  if (!idx.empty() && idx.back() + 1 != traj.size()) idx.push_back(traj.size() - 1);

  std::ostringstream o;
  o << "{\"scenario\":" << json_string(s.name) << ",\"description\":" << json_string(s.description)
    << ",\"class\":\"" << volcontact::to_string(s.theory_class) << "\",\"grounds\":[";
  for (std::size_t g = 0; g < s.grounds.size(); ++g) {
    o << (g ? "," : "") << "{\"surface\":" << json_string(s.grounds[g].shape.surface.generic_string())
      << ",\"pose\":" << pose_json(s.grounds[g].X_WG) << "}";
  }
  o << "],\"t\":[";
  for (std::size_t k = 0; k < idx.size(); ++k) o << (k ? "," : "") << short_number(traj.times()[idx[k]]);
  o << "],\"bodies\":[";
  for (int b = 0; b < traj.num_bodies(); ++b) {
    o << (b ? "," : "") << "{\"name\":" << json_string(s.bodies[b].name)
      << ",\"surface\":" << json_string(s.bodies[b].shape.surface.generic_string()) << ",\"poses\":[";
    for (std::size_t k = 0; k < idx.size(); ++k) o << (k ? "," : "") << pose_json(traj.pose(idx[k], b));
    o << "]}";
  }
  o << "],\"series\":{";
  for (std::size_t j = 0; j < report.series.size(); ++j) {
    o << (j ? "," : "") << json_string(report.series[j].name) << ":[";
    for (std::size_t k = 0; k < idx.size(); ++k) o << (k ? "," : "") << short_number(report.series[j].values[idx[k]]);
    o << "]";
  }
  o << "},\"error\":" << short_number(report.error) << ",\"scalars\":{";
  for (std::size_t j = 0; j < report.scalars.size(); ++j) {
    o << (j ? "," : "") << json_string(report.scalars[j].first) << ":" << short_number(report.scalars[j].second);
  }
  o << "}}\n";
  fs::create_directories(path.parent_path());
  std::ofstream(path) << o.str();
}

double median(std::vector<double> v) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

// Smallest incline at which the block keeps sliding (v > 1 mm/s after 2 s).
double onset_angle(const volcontact::Engine& engine, const volcontact::RunOptions& options, const volcontact::ScenarioOptions& so) {
  double lo = 20.0, hi = 30.0;
  while (hi - lo > 0.01) {
    const double mid = 0.5 * (lo + hi);
    const volcontact::Scenario s = volcontact::scenarios::sliding_block(so, mid, 2.0);
    const volcontact::RunResult r = engine.run(s, options);
    const bool slides = r.trajectory.twist(r.trajectory.size() - 1, 0).v.x() > 1e-3;
    (slides ? hi : lo) = mid;
  }
  return 0.5 * (lo + hi);
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse(argc, argv);
  if (args.list) {
    for (const auto& e : volcontact::catalog()) std::cout << e.name << '\n';
    return 0;
  }

  std::vector<std::string> wanted;
  if (args.scenario == "all") {
    for (const auto& e : volcontact::catalog()) wanted.push_back(e.name);
  } else {
    std::istringstream list(args.scenario);
    for (std::string name; std::getline(list, name, ',');) wanted.push_back(name);
  }
  const auto engines = volcontact::available_engines();
  if (args.list_native) {
    for (const auto& engine : engines) {
      std::cout << engine->name() << ':';
      for (const std::string& model : engine->native_models()) std::cout << ' ' << model;
      std::cout << '\n';
    }
    return 0;
  }
  bool any = false;
  for (const auto& engine : engines) any = any || !methods(*engine, args.contact).empty();
  if (!any) usage();

  std::printf("%-8s %-14s %-9s %-10s %-8s %10s %10s %9s   metrics\n", "engine", "contact", "scenario", "variant",
              "status", "wall [s]", "contact[s]", "RT ratio");

  const std::set<std::string> done = args.resume ? done_runs(args.csv) : std::set<std::string>();
  std::map<std::string, bool> curved;  // scenario -> its assets depend on the resolution
  for (const Config& config : configs(args)) {
    const volcontact::ScenarioOptions scenario_options{
        .data_dir = args.data, .resolution = config.mesh, .stiffness_scale = config.stiffness_scale};
    const std::vector<Item> items = make_items(wanted, config, args.data, args.max_duration);

    for (const auto& engine : engines) {
      if (args.engine != "all" && args.engine != engine->name()) continue;
      for (const Method& chosen : methods(*engine, args.contact)) {
        const volcontact::ContactMethod method = chosen.choice.contact;
        const std::string& label = chosen.choice.label;
        volcontact::RunOptions options{.dt = config.dt,
                               .contact = method,
                               .record = true,
                               .native_model = chosen.choice.native_model,
                               .native_primitives = args.native_primitives,
                               .native_resolution = args.native_resolution,
                               .time_limit_seconds = args.time_limit};
        std::string tag = std::string(engine->name()) + "-" + label;
        if (method == volcontact::ContactMethod::kNative && args.native_primitives) {
          tag += "-prim" + json_number(args.native_resolution);
        }
        if (config.name.empty()) {
          tag += "-dt" + json_number(config.dt);
          if (config.mesh != volcontact::Resolution::kBase) tag += "-seg" + std::to_string(static_cast<int>(config.mesh));
          if (config.perturbation) tag += "-perturbed" + std::to_string(config.perturbation);
        } else {
          tag += "-" + config.name;
        }
        const fs::path dir = args.out / tag;
        if (!args.out.empty()) fs::create_directories(dir);

        std::ostringstream json;
        json << "{\n  \"engine\": \"" << engine->name() << "\",\n  \"contact\": \"" << label
             << "\",\n  \"variant\": \"" << config.name << "\",\n  \"dt\": " << json_number(config.dt)
             << ",\n  \"segments\": " << static_cast<int>(config.mesh)
             << ",\n  \"stiffness_scale\": " << json_number(config.stiffness_scale)
             << ",\n  \"perturbation\": " << config.perturbation << ",\n  \"native_primitives\": "
             << (args.native_primitives ? "true" : "false") << ",\n  \"scenarios\": {";

        std::size_t emitted = 0;
        for (std::size_t k = 0; k < items.size(); ++k) {
          const Item& item = items[k];
          const std::string& name = item.name;
          std::optional<volcontact::Scenario> scenario = item.scenario;
          if (scenario && chosen.thesis) {
            for (volcontact::ContactSpec& c : scenario->contacts) {
              c.gonthier.friction_model = volcontact::GonthierParams::FrictionModel::kThesis2018;
            }
          }
          if (scenario) {
            for (volcontact::ContactSpec& c : scenario->contacts) c.distributed.friction_quadrature = chosen.quadrature;
          }
          if (done.count(name + "|" + std::string(engine->name()) + "|" + label + "|" + config.name)) continue;
          const volcontact::Scenario* s = scenario ? &*scenario : nullptr;
          const double duration = s ? s->duration : NAN;
          if (!curved.count(name)) {
            try {
              curved[name] = volcontact::depends_on_resolution(name, args.data);
            } catch (const std::exception&) {
              curved[name] = false;
            }
          }

          // A run that throws or returns nan is reported; the battery goes on.
          Row row{.engine = engine->name(),
                  .contact = label,
                  .scenario = name,
                  .variant = config.name,
                  .status = "failed",
                  .dt = config.dt,
                  .segments = curved[name] ? static_cast<int>(config.mesh) : 0,
                  .stiffness_scale = config.stiffness_scale,
                  .theory_class = s ? std::string(volcontact::to_string(s->theory_class)) : ""};
          std::vector<double> wall, contact;
          std::optional<double> onset;
          std::vector<std::pair<std::string, double>> scalars;
          double contact_points = 0.0;
          const auto attempt = std::chrono::steady_clock::now();
          try {
            if (!s) throw std::runtime_error(item.error);
            const volcontact::RunResult recorded = engine->run(*s, options);
            const volcontact::MetricReport report = s->metrics(recorded.trajectory);
            if (!args.out.empty()) write_csv(dir / (name + ".csv"), recorded.trajectory, report);
            if (!args.traj.empty()) {
              write_traj(args.traj / name / (config.name.empty() ? "default" : config.name) /
                             (std::string(engine->name()) + "-" + label + ".json"),
                         *s, recorded.trajectory, report);
            }
            row.error = report.error;
            scalars = report.scalars;
            row.steps = recorded.stats.steps;
            contact_points = recorded.stats.native_contact_points;
            // The recorded run is the first timing; repeats are timed without recording.
            wall.push_back(recorded.stats.wall_seconds);
            contact.push_back(recorded.stats.contact_seconds);
            volcontact::RunOptions timing = options;
            timing.record = false;
            for (int i = 1; i < args.repeat; ++i) {
              const volcontact::RunStats st = engine->run(*s, timing).stats;
              wall.push_back(st.wall_seconds);
              contact.push_back(st.contact_seconds);
            }
            if (args.onset && name == "block") {
              onset = onset_angle(*engine, options, scenario_options);
            }
            row.status = std::isfinite(row.error) ? "ok" : "nan";
          } catch (const volcontact::RunTimeout& e) {
            row.status = "timeout";
            std::cerr << engine->name() << " " << label << " " << name << ": " << e.what() << "\n";
          } catch (const std::exception& e) {
            std::cerr << engine->name() << " " << label << " " << name << ": " << e.what() << "\n";
          }

          const double wall_med = median(wall), contact_med = median(contact);
          const double wall_min = wall.empty() ? NAN : *std::min_element(wall.begin(), wall.end());
          const double ratio = wall_med / duration;
          row.ratio = ratio;
          row.wall = wall.empty() ? std::chrono::duration<double>(std::chrono::steady_clock::now() - attempt).count()
                                  : wall_med;

          std::printf("%-8s %-14s %-9s %-10s %-8s %10.4f %10.4f %9.4f  ", std::string(engine->name()).c_str(),
                      label.c_str(), name.c_str(), config.name.c_str(), row.status.c_str(), row.wall, contact_med,
                      ratio);
          std::printf(" error=%.3g", row.error);
          if (method == volcontact::ContactMethod::kNative) std::printf(" contact_points=%.3g", contact_points);
          for (const auto& [sname, value] : scalars) std::printf(" %s=%.3g", sname.c_str(), value);
          if (onset) std::printf(" onset_angle_deg=%.3f", *onset);
          std::printf("\n");

          if (!args.csv.empty()) append_row(args.csv, row);
          if (!args.terms.empty()) append_terms(args.terms, row, scalars);

          json << (emitted++ ? "," : "") << "\n    \"" << name << "\": {\n      \"class\": \"" << row.theory_class
               << "\",\n      \"status\": \"" << row.status << "\",\n      \"duration_s\": "
               << json_number(duration) << ",\n      \"steps\": " << row.steps
               << ",\n      \"wall_median_s\": " << json_number(wall_med)
               << ",\n      \"wall_min_s\": " << json_number(wall_min)
               << ",\n      \"contact_median_s\": " << json_number(contact_med)
               << ",\n      \"realtime_ratio\": " << json_number(ratio);
          json << ",\n      \"error\": " << json_number(row.error);
          if (onset) json << ",\n      \"onset_angle_deg\": " << json_number(*onset);
          for (const auto& [sname, value] : scalars) json << ",\n      \"" << sname << "\": " << json_number(value);
          json << "\n    }";
        }
        json << "\n  }\n}\n";
        if (!args.out.empty()) std::ofstream(dir / "summary.json") << json.str();
      }
    }
  }
  return 0;
}
