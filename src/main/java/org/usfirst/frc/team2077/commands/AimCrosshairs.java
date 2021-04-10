/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class AimCrosshairs extends CommandBase {

  private boolean _corse = false;//(toggable) For WSAD control
  private boolean _WASD = false;//(Set uppon construction) - false (default) is stick mode, true is for WSAD mode
  private static final double _slowSpeed = .5;// the % value from 0-1 of the simulated value from it it was a stick for !core mode
  private static final double _fastSpeed = .97;// the % value from 0-1 of the simulated value from it it was a stick for core mode

  //Mode constructor as orignal - uses stick to drive
  public AimCrosshairs() {
    addRequirements(robot_.crosshairs_);
  }

  //Mode constructor as settable on construct - uses WASD to drive if true, stick if false
  public AimCrosshairs(boolean mode_) {
    _WASD = mode_;
    addRequirements(robot_.crosshairs_);
  }


  @Override
  public void execute() {
    double x = 0;
    double y = 0;
    double changeSpeed;//This referes to the speed in which the white line on the vision moves

    if(!_WASD){
      x = DriveStation.adjustInputSensitivity(robot_.driveStation_.secondaryStick_.getX(), .2, 2.5);
      y = DriveStation.adjustInputSensitivity(-robot_.driveStation_.secondaryStick_.getY(), .2, 2.5);
    }else{//If WASD mode
      if(_corse)
        changeSpeed = _fastSpeed;
      else
        changeSpeed = _slowSpeed;
    
/*    W : 7     UP Y
      A : 2     Down X
      S : 8     DOWN Y
      D : 14    UP X

      ThumbTrigger : 23 */

      if(robot_.driveStation_.testingStick_.getRawButton(7)){
        y = DriveStation.adjustInputSensitivity(-changeSpeed, .2, 2.5);
      }else if(robot_.driveStation_.testingStick_.getRawButton(14)){
        y = DriveStation.adjustInputSensitivity(changeSpeed, .2, 2.5);
      }

      if(robot_.driveStation_.testingStick_.getRawButton(2)){
        x = DriveStation.adjustInputSensitivity(-changeSpeed, .2, 2.5);
      }else if(robot_.driveStation_.testingStick_.getRawButton(8)){
        x = DriveStation.adjustInputSensitivity(changeSpeed, .2, 2.5);
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
