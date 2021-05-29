/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.DriveStation;

import java.util.EnumMap;

public class AimCrosshairs extends CommandBase {
    private boolean USE_WASD = true;
  private boolean fast = false;//(toggable) For WSAD control
  private static final double _slowSpeed = .5;// the % value from 0-1 of the simulated value from it it was a stick for !core mode
  private static final double _fastSpeed = .97;// the % value from 0-1 of the simulated value from it it was a stick for core mode

  private enum SimulatedKey {
    W(true, 1),
    A(false, 1),
    S(true, -1),
    D(false, -1)
    ;

    private final double adjustment;
    private final boolean yAxis;
    SimulatedKey(boolean yAxis, double adjustment) {
      this.yAxis = yAxis;
      this.adjustment = adjustment;
    }
  }

  private final EnumMap<SimulatedKey, JoystickButton> targets= new EnumMap<>(SimulatedKey.class);
  private final Joystick targetStick;

  //Mode constructor as settable on construct - uses WASD to drive if true, stick if false
  public AimCrosshairs(Joystick secondaryStick, Joystick testingStick) {
    addRequirements(robot_.crosshairs_);
    if(USE_WASD) {
      targets.put(SimulatedKey.W, new JoystickButton(testingStick, 2));
      targets.put(SimulatedKey.A, new JoystickButton(testingStick, 5));
      targets.put(SimulatedKey.S, new JoystickButton(testingStick, 6));
      targets.put(SimulatedKey.D, new JoystickButton(testingStick, 7));

      targetStick = null;
    } else {
      targetStick = secondaryStick;
    }
  }


  @Override
  public void execute() {
    double x = 0;
    double y = 0;

    if (USE_WASD) {
      for(SimulatedKey m : SimulatedKey.values()) {
        if(targets.get(m).get()) {
          if(m.yAxis) y += m.adjustment;
          else x += m.adjustment;
        }
      }
    } else {
      x = DriveStation.adjustInputSensitivity(targetStick.getX(), .2, 2.5);
      y = DriveStation.adjustInputSensitivity(targetStick.getY(), .2, 2.5);
    }
    double[] ar = robot_.crosshairs_.get();
    double azimuth = ar[0];
    double range = ar[1];


    if (Math.abs(x) > .01) {
      azimuth += x * robot_.constants_.AZIMUTH_CROSSHAIR_SENSITIVITY;
      double fov2 = robot_.crosshairs_.getHorizontalFOV() / 2.;
      azimuth = Math.max(-fov2, Math.min(fov2, azimuth)); //keeps crosshairs above half height
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
