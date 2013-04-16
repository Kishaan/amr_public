#include "diff_velocity_controller.h"

DiffVelocityController::DiffVelocityController(double l_max_vel, double l_tolerance,
                                               double a_max_vel, double a_tolerance)
: l_max_vel_(l_max_vel)
, l_tolerance_(l_tolerance)
, a_max_vel_(a_max_vel)
, a_tolerance_(a_tolerance)
{
}

void DiffVelocityController::setTargetPose(const Pose& pose)
{
  target_pose_ = pose;
  linear_complete_ = false;
  angular_complete_ = false;
}

bool DiffVelocityController::isTargetReached() const
{
  return linear_complete_ & angular_complete_;
}

Velocity DiffVelocityController::computeVelocity(const Pose& actual_pose)
{
  //========================= YOUR CODE HERE =========================
  // Instructions: compute the velocity based on the target pose and
  //               the actual pose. Update the bool flags
  //               linear_complete_ and angular_complete_ to indicate
  //               whether a corresponding component of the movement
  //               is done.
  //
  // Hint: use the static helper functions getDistance() and
  //       getShortestAngle() implemented in the VelocityController
  //       class.


  //==================================================================
}

