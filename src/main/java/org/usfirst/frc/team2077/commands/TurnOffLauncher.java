/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.robot_;

public class TurnOffLauncher extends CommandBase {

  public TurnOffLauncher() {
    addRequirements(robot_.testLauncher_);
  }

  @Override
  public void initialize() {
    robot_.testLauncher_.setRunning(false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
