/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class LauncherSpinTest extends CommandBase {

  private final double increment_;

  public LauncherSpinTest(double increment) {
    addRequirements(robot_.testLauncher_);
    increment_ = increment;
  }

  @Override
  public void initialize() {
    double i = Math.abs(increment_);
    double rpm = Math.round(robot_.testLauncher_.launcherRPM_ / i) * i + increment_;
    robot_.testLauncher_.setLauncherRPM(Math.max(i, Math.min(robot_.testLauncher_.launcherMaxRPM_, rpm)));
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
