#include <string>
#include <vector>

#ifdef GTEST_DONT_DEFINE_TEST
#undef GTEST_DONT_DEFINE_TEST
#endif
#include <gtest/gtest.h>
#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "examples/box-repositioning/parameters/box_repositioning_config.h"

namespace dairlib {
namespace examples {
namespace box_repositioning {
namespace {

TEST(BoxRepositioningConfigTest, LoadsExpectedValues) {
  const std::string config_path = ::dairlib::FindResourceOrThrow(
      "examples/box-repositioning/config/box_repositioning_config.yaml");
  BoxRepositioningConfig config =
      drake::yaml::LoadYamlFile<BoxRepositioningConfig>(config_path);

  EXPECT_EQ(config.scene_index, 1);
  EXPECT_FALSE(config.include_end_effector_orientation);

  const std::vector<std::string> expected_file_names{
      "1_testing.yaml", "2_supports.yaml", "3_rotated_supports.yaml",
      "4_wall.yaml"};
  EXPECT_EQ(config.file_names, expected_file_names);
  EXPECT_LT(config.scene_index, static_cast<int>(config.file_names.size()));

  EXPECT_EQ(config.c3_controller_options_directory,
            "examples/plate-balancing/config/C3/controller-c3plus-options/");
  EXPECT_EQ(config.c3_scene_config_directory,
            "examples/plate-balancing/config/C3/scene-config/");
  EXPECT_EQ(config.c3_osqp_settings_file,
            "examples/plate-balancing/config/C3/solver_settings.yaml");
  EXPECT_EQ(config.c3_actor_trajectory_generator_config,
            "examples/plate-balancing/config/C3/trajectory-generator-config/"
            "actor_trajectory_generator.yaml");
  EXPECT_EQ(config.c3_object_trajectory_generator_config,
            "examples/plate-balancing/config/C3/trajectory-generator-config/"
            "object_trajectory_generator.yaml");
  EXPECT_EQ(config.simulation_scene_config_directory,
            "examples/plate-balancing/config/Simulation/scene-config/");
  EXPECT_EQ(config.simulation_config_file,
            "examples/plate-balancing/config/Simulation/simulation_config.yaml");
  EXPECT_EQ(config.osc_contoller_config_file,
            "examples/plate-balancing/config/OSC/controller_config.yaml");
  EXPECT_EQ(config.osc_osqp_setting_file,
            "examples/plate-balancing/config/OSC/solver_settings.yaml");
  EXPECT_EQ(config.lcm_hardware_settings_file,
            "examples/plate-balancing/config/LCM/lcm_channels_hardware.yaml");
  EXPECT_EQ(config.lcm_simulation_settings_file,
            "examples/plate-balancing/config/LCM/lcm_channels_simulation.yaml");
  EXPECT_EQ(config.box_repositioning_target_config_file,
            "examples/box-repositioning/config/box_repositioning_target_config.yaml");

  EXPECT_EQ(config.get_c3_controller_option_file(),
            config.c3_controller_options_directory +
                config.file_names[config.scene_index]);
  EXPECT_EQ(config.get_c3_scene_config_file(),
            config.c3_scene_config_directory +
                config.file_names[config.scene_index]);
  EXPECT_EQ(config.get_simulation_scene_config_file(),
            config.simulation_scene_config_directory +
                config.file_names[config.scene_index]);
}

}  // namespace
}  // namespace box_repositioning
}  // namespace examples
}  // namespace dairlib
