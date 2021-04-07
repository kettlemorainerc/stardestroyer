/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.CLOCKWISE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.drivetrain.MecanumMath;

public class SteerToCrosshairs extends CommandBase {

  private double fast_;
  private double slow_;
  private double deceleration_;
  
  public SteerToCrosshairs() {
    addRequirements(robot_.heading_);
    fast_ = robot_.chassis_.getMaximumVelocity()[2];
    slow_ = robot_.chassis_.getMinimumVelocity()[2];
    deceleration_ = robot_.chassis_.getAccelerationLimits()[2][1];
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double crosshairHeading = robot_.crosshairs_.getHeading(); // angle from robot to target
    double rotationVelocity = robot_.chassis_.getVelocityCalculated().get(CLOCKWISE); // current rotation speed

    if (Math.abs(crosshairHeading) < .5) { // considered "on target"  // TODO: Configure. Also consider range.
      robot_.chassis_.setRotation(0);
      return;
    }

    if (Math.signum(rotationVelocity) * Math.signum(crosshairHeading) == -1.0) { // rotating away from target
      robot_.chassis_.setRotation(-rotationVelocity/2.); // reverse and slow
      System.out.println("**************** " + rotationVelocity);
      return;
    }
    
    double stopRotation = .5 * rotationVelocity*rotationVelocity / deceleration_; // stopping distance at deceleration limit from physics
    double pad = Math.max(stopRotation*.1, Math.abs(rotationVelocity)*.08); // overestimate stopping distance by 5% or distance traveled in two .02 second control cycles
    robot_.chassis_.setRotation((Math.abs(crosshairHeading)>(stopRotation+pad) ? fast_ : slow_) * Math.signum(crosshairHeading)); // start slowing a bit early, creep to end
  }

  @Override
  public void end(boolean interrupted) {
    robot_.chassis_.setRotation(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
