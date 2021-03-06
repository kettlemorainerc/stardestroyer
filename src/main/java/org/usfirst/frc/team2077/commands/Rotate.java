/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static org.usfirst.frc.team2077.Robot.*;

public class Rotate extends CommandBase {
  private double rotation_;
  private double finalHeading_;
  private double remainingRotation_;
  private double fast_;
  private double slow_;
  private double deceleration_;

  public Rotate(double rotation) {
    addRequirements(robot_.heading_);
    rotation_ = rotation;
  }

  public void initialize() {
    finalHeading_ = robot_.chassis_.getPosition()[2] + rotation_; // target heading
    fast_ = robot_.chassis_.getMaximumVelocity()[2] * Math.signum(rotation_); // full speed in desired direction
    slow_ = robot_.chassis_.getMinimumVelocity()[2] * Math.signum(rotation_); // creep in desired direction to ensure target is reached
    deceleration_ = robot_.chassis_.getAccelerationLimits()[2][1]; // in degrees/second/second
  }

  @Override
  public void execute() {
    // Run pedal to the metal until last possible moment, then slam on the brakes, letting DriveChassis handle acceleration and deceleration.
    double velocity = robot_.chassis_.getVelocityCalculated()[2]; // current velocity
    double stopRotation = .5 * velocity*velocity / deceleration_; // stopping distance at deceleration limit from physics
    double pad = Math.max(stopRotation*.05, Math.abs(velocity)*.04); // overestimate stopping distance by 5% or distance traveled in two .02 second control cycles
    robot_.chassis_.setRotation(Math.abs(remainingRotation_)>(stopRotation+pad) ? fast_ : slow_); // start slowing a bit early, creep to end
  }

  @Override
  public void end(boolean interrupted) {
    robot_.chassis_.setRotation(0);
  }

  @Override
  public boolean isFinished() {
    // done when remaining rotation goes past zero
    remainingRotation_ = finalHeading_ - robot_.chassis_.getPosition()[2];
    return Math.signum(remainingRotation_) != Math.signum(rotation_);
  }
}
