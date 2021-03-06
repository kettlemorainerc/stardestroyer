/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

// Don't use if secondary stick is otherwise assigned! Use LauncherScrewTest with buttons.
@Deprecated
public class AngleLauncher extends CommandBase {
  public AngleLauncher() {
    addRequirements(robot_.testLauncher_);
  }

  @Override
  public void execute() {
    robot_.testLauncher_.setScrewPosition01(-robot_.driveStation_.secondaryStick_.getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
