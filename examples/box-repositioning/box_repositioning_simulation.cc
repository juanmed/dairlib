#include <math.h>

#include <vector>

#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/visualization/visualization_config_functions.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "examples/plate-balancing/parameters/lcm_channel_config.h"
#include "examples/plate-balancing/parameters/simulation_config.h"
#include "examples/plate-balancing/parameters/simulation_scene_config.h"
#include "examples/plate-balancing/systems/external_force_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/lcmt_systems/common.h"
#include "systems/lcmt_systems/object_state_systems.h"
#include "systems/primitives/radio_parser.h"
#include "systems/system_utils.h"

#include "parameters/box_repositioning_config.h"


namespace dairlib {

using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using systems::RadioToVector;
using systems::SubvectorPassThrough;
using systems::lcmt_systems::AddActuationRecieverAndStateSenderLcm;
using systems::lcmt_systems::ObjectStateGenerator;

using examples::plate_balancing::SimulationConfig;
using examples::plate_balancing::LcmChannelConfig;
using examples::plate_balancing::SimulationSceneConfig;

namespace examples {
namespace box_repositioning {

// Loads the main configuration file for the plate balancing simulation.
DEFINE_string(box_repositioning_config,
              "examples/box-repositioning/config/box_repositioning_config.yaml",
              "Controller settings such as channels.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load simulation and controller parameters from YAML files.
  BoxRepositioningConfig main_config =
      drake::yaml::LoadYamlFile<BoxRepositioningConfig>(
          FLAGS_box_repositioning_config);
  SimulationConfig sim_params = drake::yaml::LoadYamlFile<SimulationConfig>(
      main_config.simulation_config_file);
  LcmChannelConfig lcm_channel_params =
      drake::yaml::LoadYamlFile<LcmChannelConfig>(
          main_config.lcm_simulation_settings_file);
  SimulationSceneConfig scene_params =
      drake::yaml::LoadYamlFile<SimulationSceneConfig>(
          main_config.get_simulation_scene_config_file());

  return 0;
}

} // namespace examples
} // namespace box_repositioning
} // namespace dairlib

// Entry point for the simulation.
int main(int argc, char* argv[]) {
  dairlib::examples::box_repositioning::DoMain(argc, argv);
}
