/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class AimCrosshairs extends CommandBase {

  private boolean fast = false;//(toggable) For WSAD control
  private boolean useKeyboard;//(Set uppon construction) - false (default) is stick mode, true is for WSAD mode
  private static final double _slowSpeed = .5;// the % value from 0-1 of the simulated value from it it was a stick for !core mode
  private static final double _fastSpeed = .97;// the % value from 0-1 of the simulated value from it it was a stick for core mode

  //Mode constructor as orignal - uses stick to drive
  public AimCrosshairs() {
    this(false);
  }

  //Mode constructor as settable on construct - uses WASD to drive if true, stick if false
  public AimCrosshairs(boolean useKeyboard) {
    this.useKeyboard = useKeyboard;
    addRequirements(robot_.crosshairs_);
  }


  @Override
  public void execute() {
    double x = 0;
    double y = 0;

    if(!useKeyboard) {
      x = DriveStation.adjustInputSensitivity(robot_.driveStation_.secondaryStick_.getX(), .2, 2.5);
      y = DriveStation.adjustInputSensitivity(-robot_.driveStation_.secondaryStick_.getY(), .2, 2.5);
    } else {
      double manualChange = fast ? _fastSpeed : _slowSpeed;
      final int W = 7, A = 2, S = 8, D = 14, THUMB_TRIGGER = 23;

      if(robot_.driveStation_.testingStick_.getRawButton(W)){
        y = DriveStation.adjustInputSensitivity(-manualChange, .2, 2.5);
      }else if(robot_.driveStation_.testingStick_.getRawButton(S)){
        y = DriveStation.adjustInputSensitivity(manualChange, .2, 2.5);
      }

      if(robot_.driveStation_.testingStick_.getRawButton(A)){
        x = DriveStation.adjustInputSensitivity(-manualChange, .2, 2.5);
      }else if(robot_.driveStation_.testingStick_.getRawButton(D)){
        x = DriveStation.adjustInputSensitivity(manualChange, .2, 2.5);
      }
      
    }

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
