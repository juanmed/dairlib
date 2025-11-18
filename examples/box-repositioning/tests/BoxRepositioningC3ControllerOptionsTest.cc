#include <initializer_list>
#include <string>
#include <vector>

#include <Eigen/Dense>

#ifdef GTEST_DONT_DEFINE_TEST
#undef GTEST_DONT_DEFINE_TEST
#endif
#include <gtest/gtest.h>
#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "examples/box-repositioning/parameters/box_repositioning_c3_controller_options.h"
#include "examples/box-repositioning/parameters/box_repositioning_config.h"

namespace dairlib {
namespace examples {
namespace box_repositioning {
namespace {

TEST(BoxRepositioningC3ControllerOptionsTest, LoadsExpectedValues) {
  const std::string config_path = ::dairlib::FindResourceOrThrow(
      "examples/box-repositioning/config/box_repositioning_config.yaml");
  BoxRepositioningConfig config =
      drake::yaml::LoadYamlFile<BoxRepositioningConfig>(config_path);

  const std::string options_path = ::dairlib::FindResourceOrThrow(
      config.get_c3_controller_option_file());
  BoxRepositioningC3ControllerOptions options =
      drake::yaml::LoadYamlFile<BoxRepositioningC3ControllerOptions>(
          options_path);

  EXPECT_EQ(options.projection_type, "C3+");
  EXPECT_DOUBLE_EQ(options.solve_time_filter_alpha, 0.95);
  EXPECT_DOUBLE_EQ(options.publish_frequency, 0.0);
  EXPECT_TRUE(options.state_prediction_joints.empty());

  ASSERT_EQ(options.u_horizontal_limits.size(), 2);
  EXPECT_DOUBLE_EQ(options.u_horizontal_limits[0], -50.0);
  EXPECT_DOUBLE_EQ(options.u_horizontal_limits[1], 50.0);

  ASSERT_EQ(options.u_vertical_limits.size(), 2);
  EXPECT_DOUBLE_EQ(options.u_vertical_limits[0], -50.0);
  EXPECT_DOUBLE_EQ(options.u_vertical_limits[1], 50.0);

  const auto make_vector = [](std::initializer_list<double> values) {
    Eigen::VectorXd vec(values.size());
    int index = 0;
    for (double value : values) {
      vec[index++] = value;
    }
    return vec;
  };
  const std::vector<Eigen::VectorXd> expected_workspace_limits = {
      make_vector({1.0, 0.0, 0.0, 0.4, 0.6}),
      make_vector({0.0, 1.0, 0.0, -0.2, 0.2}),
      make_vector({0.0, 0.0, 1.0, 0.3, 0.6}),
  };
  ASSERT_EQ(options.workspace_limits.size(),
            expected_workspace_limits.size());
  for (size_t i = 0; i < expected_workspace_limits.size(); ++i) {
    EXPECT_TRUE(options.workspace_limits[i].isApprox(
        expected_workspace_limits[i], 1e-12));
  }
  EXPECT_DOUBLE_EQ(options.workspace_margins, 0.05);

  EXPECT_EQ(options.lcs_factory_options.contact_model, "stewart_and_trinkle");
  EXPECT_EQ(options.lcs_factory_options.num_friction_directions, 2);
  EXPECT_EQ(options.lcs_factory_options.num_contacts, 3);
  EXPECT_DOUBLE_EQ(options.lcs_factory_options.dt, 0.05);

  EXPECT_FALSE(options.c3_options.warm_start);
  EXPECT_FALSE(options.c3_options.end_on_qp_step);
  ASSERT_TRUE(options.c3_options.penalize_input_change.has_value());
  EXPECT_TRUE(*options.c3_options.penalize_input_change);
  EXPECT_EQ(options.c3_options.num_threads, 5);
  EXPECT_EQ(options.c3_options.admm_iter, 2);
  EXPECT_DOUBLE_EQ(options.c3_options.gamma, 1.0);
  EXPECT_FLOAT_EQ(options.c3_options.rho_scale, 4.0f);
  EXPECT_DOUBLE_EQ(options.c3_options.w_Q, 50.0);
  EXPECT_DOUBLE_EQ(options.c3_options.w_R, 25.0);
  EXPECT_DOUBLE_EQ(options.c3_options.w_G, 0.1);
  EXPECT_DOUBLE_EQ(options.c3_options.w_U, 0.5);
  EXPECT_EQ(options.c3_options.q_vector.size(), 19);
  EXPECT_EQ(options.c3_options.r_vector.size(), 3);
}

}  // namespace
}  // namespace box_repositioning
}  // namespace examples
}  // namespace dairlib
