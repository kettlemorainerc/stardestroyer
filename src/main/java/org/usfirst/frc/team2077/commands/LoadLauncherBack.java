/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;


public class LoadLauncherBack extends CommandBase {
  double timeDif = 130;
  double time;
  public LoadLauncherBack() {
    addRequirements(robot_.testLauncher_);
  }

  @Override
  public void initialize() {
    time = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if (System.currentTimeMillis() < time + timeDif) {
      robot_.testLauncher_.runLoader(-1.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    robot_.testLauncher_.stopLoader();
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() > time + timeDif) {
      return true;
    }
    return false;
  }
}
