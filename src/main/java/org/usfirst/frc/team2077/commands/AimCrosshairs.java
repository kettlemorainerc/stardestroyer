/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.subsystems.controller.ControllerBinding;
import org.usfirst.frc.team2077.subsystems.controller.ControllerBinding.Axis;

import static org.usfirst.frc.team2077.Robot.*;

public class AimCrosshairs extends CommandBase {
  private static final boolean USE_WASD = true;
  private static final double MAX_STEP = 1;
  private boolean fast = false;//(toggable) For WSAD control
  private static final double _slowSpeed = .5;// the % value from 0-1 of the simulated value from it it was a stick for !core mode
  private static final double _fastSpeed = .97;// the % value from 0-1 of the simulated value from it it was a stick for core mode

  private enum SimulatedKey {
    W(true, 1),
    A(false, -1),
    S(true, -1),
    D(false, 1)
    ;

    private final double adjustment;
    private final boolean yAxis;
    SimulatedKey(boolean yAxis, double adjustment) {
      this.yAxis = yAxis;
      this.adjustment = adjustment;
    }
  }

  private final ControllerBinding controller;

  //Mode constructor as settable on construct - uses WASD to drive if true, stick if false
  public AimCrosshairs(ControllerBinding controller) {
    addRequirements(robot_.crosshairs_);
    this.controller = controller;
  }

  @Override
  public void execute() {
    double x = 0;
    double y = 0;

    if(controller != null) {
      x += controller.getAxis(Axis.EAST) * MAX_STEP;
      y += controller.getAxis(Axis.NORTH) * MAX_STEP;
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
