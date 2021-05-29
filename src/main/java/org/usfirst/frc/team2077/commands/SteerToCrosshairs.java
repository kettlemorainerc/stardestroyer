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

  private final double maxRPMFine;
  private final double maxRPMCoarse;
  private final double deceleration_;
  private final double angleToBeginSlowingDown;
  private final double deadZoneAngle;
  
  public SteerToCrosshairs() {
    addRequirements(robot_.heading_);
    maxRPMFine = 800;
    maxRPMCoarse = 3000;
    angleToBeginSlowingDown = 45;
    deceleration_ = robot_.chassis_.getAccelerationLimits()[2][1];
    deadZoneAngle = .7; // TODO: Configure. Also consider range.
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double crosshairHeading = robot_.crosshairs_.getHeading(); // angle from robot to target
    double rotationVelocity = robot_.chassis_.getVelocityCalculated().get(CLOCKWISE); // current rotation speed

    if (Math.abs(crosshairHeading) < deadZoneAngle) {
      robot_.chassis_.setRotation(0);
      return;
    }

    if (Math.signum(rotationVelocity) * Math.signum(crosshairHeading) == -1.0) { // rotating away from target
      robot_.chassis_.setRotation(-rotationVelocity/2.); // reverse and slow
      System.out.println("**************** " + rotationVelocity);
      return;
    }
    double speed = Math.abs(crosshairHeading) >= angleToBeginSlowingDown ?
        maxRPMCoarse :
        maxRPMFine / Math.pow((angleToBeginSlowingDown - Math.abs(crosshairHeading)), 2);
    robot_.chassis_.setRotation(speed * Math.signum(crosshairHeading));

    //double speed = fast_ / (45 - Math.abs(azimuth));
    //robot_.chassis_.setRotation(Math.abs(azimuth) >= 45 ? fast_ * Math.signum(azimuth) : speed * Math.signum(azimuth));
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
