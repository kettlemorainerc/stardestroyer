/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import static org.usfirst.frc.team2077.Robot.robot_;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SteerToCrosshairs extends CommandBase {

  private final double maxRPMFine;
  private final double maxRPMCoarse;
  private final double deceleration_;
  private final double angleToBeginSlowingDown;
  private final double deadZoneToCentered;        //In degrees
  
  public SteerToCrosshairs() {
    addRequirements(robot_.heading_);
    maxRPMFine = 800;
    maxRPMCoarse = 3000;
    angleToBeginSlowingDown = 45;
    deceleration_ = robot_.chassis_.getAccelerationLimits()[2][1];
    deadZoneToCentered = .5; //TODO: Requires tuning
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double azimuth = robot_.crosshairs_.getAzimuth(); // angle from robot to target
    double velocity = robot_.chassis_.getVelocityCalculated()[2]; // current rotation speed

    if (Math.abs(azimuth) < deadZoneToCentered) {
      robot_.chassis_.halt();
      return;
    }

    if ((Math.signum(velocity) == Math.signum(azimuth)) || velocity == 0) { // rotating away from target
      double speed = Math.abs(azimuth) >= angleToBeginSlowingDown ? maxRPMCoarse : maxRPMFine;
      robot_.chassis_.setRotation(speed * Math.signum(azimuth));
    }else{
      robot_.chassis_.setRotation(-velocity/2.); // reverse and slow
    }
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
