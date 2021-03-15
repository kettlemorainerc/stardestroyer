/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class LoadLauncher extends CommandBase {
  public LoadLauncher(){
    addRequirements(robot_.testLauncher_);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    robot_.testLauncher_.runLoader(.2);
  }

  @Override
  public void end(boolean interrupted) {
    robot_.testLauncher_.stopLoader();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
