/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class SteerToCrosshairs extends CommandBase {

  private final double maxRPMFine;
  private final double maxRPMCoarse;
  private final double angleToBeginSlowingDown;
  private final double deadZoneToCentered;        //In degrees

  public SteerToCrosshairs() {
    addRequirements(robot_.heading_);
    maxRPMFine = 800;
    maxRPMCoarse = 3000;
    angleToBeginSlowingDown = 45;
    deadZoneToCentered = .5; //TODO: Requires tuning
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double crosshairHeading = robot_.crosshairs_.getHeading(); // angle from robot to target
    double speed = Math.abs(crosshairHeading) >= angleToBeginSlowingDown ? maxRPMCoarse : maxRPMFine / Math.ceil(angleToBeginSlowingDown - Math.abs(crosshairHeading));

    if (Math.abs(crosshairHeading) < deadZoneToCentered) robot_.chassis_.halt();
    else robot_.chassis_.setRotation(speed * Math.signum(crosshairHeading));
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
