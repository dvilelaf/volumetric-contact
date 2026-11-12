#include "volcontact/engine.hpp"

#include <algorithm>
#include <stdexcept>

#ifdef VOLCONTACT_WITH_DRAKE
#include "drake_backend/drake_engine.hpp"
#endif
#ifdef VOLCONTACT_WITH_CHRONO
#include "chrono_backend/chrono_engine.hpp"
#endif
#ifdef VOLCONTACT_WITH_MUJOCO
#include "mujoco_backend/mujoco_engine.hpp"
#endif

namespace volcontact {

std::string_view to_string(ContactMethod method) {
  switch (method) {
    case ContactMethod::kGonthier: return "gonthier";
    case ContactMethod::kDistributed: return "distributed";
    case ContactMethod::kPcm: return "pcm";
    case ContactMethod::kNative: return "native";
  }
  return "unknown";
}

ForceModel force_model(ContactMethod method) {
  switch (method) {
    case ContactMethod::kDistributed: return ForceModel::kDistributed;
    case ContactMethod::kPcm: return ForceModel::kPcm;
    default: return ForceModel::kGonthier;
  }
}

std::string native_model(const Engine& engine, const RunOptions& options) {
  const std::vector<std::string> models = engine.native_models();
  if (options.native_model.empty()) return models.front();
  if (std::find(models.begin(), models.end(), options.native_model) == models.end()) {
    throw std::invalid_argument(std::string(engine.name()) + " has no native contact model '" + options.native_model +
                                "'");
  }
  return options.native_model;
}

std::vector<ContactChoice> contact_choices(const Engine& engine, std::string_view argument) {
  std::vector<ContactChoice> choices;
  for (const auto m : {ContactMethod::kGonthier, ContactMethod::kDistributed}) {
    if (argument == "all" || argument == to_string(m)) choices.push_back({m, "", std::string(to_string(m))});
  }
  if (argument == to_string(ContactMethod::kPcm)) choices.push_back({ContactMethod::kPcm, "", "pcm"});
  const std::string native = "native";
  if (argument == "all" || argument == native) choices.push_back({ContactMethod::kNative, "", native});
  if (argument.starts_with(native + ":")) {
    const std::string_view wanted = argument.substr(native.size() + 1);
    const std::vector<std::string> models = engine.native_models();
    for (const std::string& model : models) {
      if (wanted != "all" && wanted != model) continue;
      choices.push_back({ContactMethod::kNative, model, model == models.front() ? native : native + "-" + model});
    }
  }
  return choices;
}

std::vector<std::unique_ptr<Engine>> available_engines() {
  std::vector<std::unique_ptr<Engine>> engines;
#ifdef VOLCONTACT_WITH_DRAKE
  engines.push_back(make_drake_engine());
#endif
#ifdef VOLCONTACT_WITH_CHRONO
  engines.push_back(make_chrono_engine());
#endif
#ifdef VOLCONTACT_WITH_MUJOCO
  engines.push_back(make_mujoco_engine());
#endif
  return engines;
}

}  // namespace volcontact
