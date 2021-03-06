/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class AimCrosshairs extends CommandBase {

  public AimCrosshairs() {
    addRequirements(robot_.crosshairs_);
  }

  @Override
  public void execute() {
    double x = DriveStation.adjustInputSensitivity(robot_.driveStation_.secondaryStick_.getX(), .2, 2.5);
    double y = DriveStation.adjustInputSensitivity(-robot_.driveStation_.secondaryStick_.getY(), .2, 2.5);
    double[] ar = robot_.crosshairs_.get();
    double azimuth = ar[0];
    double range = ar[1];
    if (Math.abs(x) > .01) {
      azimuth += x * robot_.constants_.AZIMUTH_CROSSHAIR_SENSITIVITY;
      double fov2 = robot_.crosshairs_.getHorizontalFOV() / 2.;
      azimuth = Math.max(-fov2, Math.min(fov2, azimuth));
      }
    if (Math.abs(y) > .01) {
      range -= y * robot_.constants_.RANGE_CROSSHAIR_SENSITIVITY;
    }
    robot_.crosshairs_.set(azimuth, range);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
