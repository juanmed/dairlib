#include "box_repositioning_target_generator.h"

#include <iostream>

#include <Eigen/Geometry>

#include "dairlib/lcmt_radio_out.hpp"

using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
using systems::StateVector;
namespace examples {
namespace box_repositioning {

BoxRepositioningTargetGenerator::BoxRepositioningTargetGenerator(
    const MultibodyPlant<double>& object_plant, double end_effector_thickness,
    double target_threshold)
    : end_effector_thickness_(end_effector_thickness),
      target_threshold_(target_threshold) {
  // Declare input/output ports and initialize discrete state for FSM.
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();
  box_state_port_ =
      this->DeclareVectorInputPort(
              "box_object", StateVector<double>(object_plant.num_positions(),
                                              object_plant.num_velocities()))
          .get_index();
  end_effector_target_port_ =
      this->DeclareVectorOutputPort(
              "end_effector_target", BasicVector<double>(5),
              &BoxRepositioningTargetGenerator::CalcEndEffectorTarget)
          .get_index();
  box_target_port_ = this->DeclareVectorOutputPort(
                              "box_target", BasicVector<double>(4),
                              &BoxRepositioningTargetGenerator::CalcBoxTarget)
                          .get_index();
  box_velocity_target_port_ =
      this->DeclareVectorOutputPort(
              "box_velocity_target", BasicVector<double>(4),
              &BoxRepositioningTargetGenerator::CalcBoxVelocityTarget)
          .get_index();
  sequence_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  within_target_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  time_entered_target_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  DeclareForcedDiscreteUpdateEvent(
      &BoxRepositioningTargetGenerator::DiscreteVariableUpdate);
}

EventStatus BoxRepositioningTargetGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  // Updates the FSM state based on box position and radio input.
  const StateVector<double>* box_state =
      (StateVector<double>*)this->EvalVectorInput(context, box_state_port_);
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);

  int current_sequence = context.get_discrete_state(sequence_index_)[0];
  int within_target = context.get_discrete_state(within_target_index_)[0];
  int time_entered_target =
      context.get_discrete_state(time_entered_target_index_)[0];
  if (current_sequence == 0) {
    if ((box_state->GetPositions().tail(3) - first_target_).norm() <
        target_threshold_) {
      if (within_target ==
          0) {  // set the time of when the box first hits the target
        discrete_state->get_mutable_value(time_entered_target_index_)[0] =
            context.get_time();
      }
      discrete_state->get_mutable_value(within_target_index_)[0] = 1;
    }
    if (within_target == 1 &&
        (context.get_time() - time_entered_target) > 0.5) {
      discrete_state->get_mutable_value(within_target_index_)[0] = 0;
      discrete_state->get_mutable_value(sequence_index_)[0] = 1;
    }
  } else if (current_sequence == 1) {
    if ((box_state->GetPositions().tail(3) - second_target_).norm() <
        target_threshold_) {
      if (within_target ==
          0) {  // set the time of when the box first hits the target
        discrete_state->get_mutable_value(time_entered_target_index_)[0] =
            context.get_time();
      }
      discrete_state->get_mutable_value(within_target_index_)[0] = 1;
    }
    if (within_target == 1 &&
        (context.get_time() - time_entered_target) > delay_at_top_) {
      discrete_state->get_mutable_value(within_target_index_)[0] = 0;
      discrete_state->get_mutable_value(sequence_index_)[0] = 2;
    }
  } else if (current_sequence == 2) {
    if ((box_state->GetPositions().tail(3) - third_target_).norm() <
        target_threshold_) {
      discrete_state->get_mutable_value(sequence_index_)[0] = 3;
    }
  }
  if (current_sequence == 3 && radio_out->value()[15] < 0) {
    discrete_state->get_mutable_value(sequence_index_)[0] = 0;
  }
  return EventStatus::Succeeded();
}

void BoxRepositioningTargetGenerator::SetRemoteControlParameters(
    const Eigen::Vector4d& first_target, const Eigen::Vector4d& second_target,
    const Eigen::Vector4d& third_target, double x_scale, double y_scale,
    double z_scale) {
  first_target_ = first_target;
  second_target_ = second_target;
  third_target_ = third_target;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void BoxRepositioningTargetGenerator::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  // Computes the end effector target position based on FSM state and radio
  // input.
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);

  VectorXd end_effector_position = first_target_;
  if (context.get_discrete_state(sequence_index_)[0] == 1) {
    end_effector_position = second_target_;
  } else if (context.get_discrete_state(sequence_index_)[0] == 2 ||
             context.get_discrete_state(sequence_index_)[0] == 3) {
    end_effector_position = third_target_;
  }
  end_effector_position[2] -= end_effector_thickness_;
  if (radio_out->value()[13] > 0) {
    end_effector_position(0) += radio_out->value()[0] * x_scale_;
    end_effector_position(1) += radio_out->value()[1] * y_scale_;
    end_effector_position(2) += radio_out->value()[2] * z_scale_;
  }
  if (end_effector_position[0] > 0.6) {
    end_effector_position[0] = 0.6;
  }
  target->SetFromVector(end_effector_position);
}

void BoxRepositioningTargetGenerator::CalcBoxTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const{
  // Computes the box target pose based on FSM state and radio input.
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  VectorXd target_box_state = VectorXd::Zero(4);
  VectorXd box_position = first_target_;

  if (context.get_discrete_state(sequence_index_)[0] == 1) {
    box_position = second_target_;
  } else if (context.get_discrete_state(sequence_index_)[0] == 2 ||
             context.get_discrete_state(sequence_index_)[0] == 3) {
    box_position = third_target_;
  }
  if (radio_out->value()[13] > 0) {
    box_position(0) += radio_out->value()[0] * x_scale_;
    box_position(1) += radio_out->value()[1] * y_scale_;
    box_position(2) += radio_out->value()[2] * z_scale_;
  }
  target_box_state << box_position;
  target->SetFromVector(target_box_state);
}

void BoxRepositioningTargetGenerator::CalcBoxVelocityTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  // Computes the box velocity target (orientation error).
  const StateVector<double>* box_state =
      (StateVector<double>*)this->EvalVectorInput(context, box_state_port_);
  const Eigen::Vector4d& q = box_state->GetPositions();

  VectorXd target_box_error = VectorXd::Zero(4);
  target_box_error = target_box_state_ - q;
  target->SetFromVector(target_box_error);
}

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib