/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

/**
 * Toggles launcher wheels on or off each time it's executed.
 * If the invoking button is held down when turning on, the wheel speed
 * may be adjusted with the secondary joystick Z dial.
 */
public class LauncherSpinTestZ extends CommandBase {
  
  public LauncherSpinTestZ() {
    addRequirements(robot_.testLauncher_);
  }

  @Override
  public void initialize() {
    robot_.testLauncher_.setRunning(robot_.testLauncher_.isRunning());
  }

  @Override
  public void execute() {
    double z = (robot_.driveStation_.secondaryStick_.getThrottle() + 1.)/ 2.;
    z = .1 + .8*z; // scale from 10% to 100%
    robot_.testLauncher_.setLauncherRPM(z * robot_.testLauncher_.launcherMaxRPM_);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
