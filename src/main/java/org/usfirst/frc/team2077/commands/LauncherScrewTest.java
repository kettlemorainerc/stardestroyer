/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

/**
 * Move launcher screw up or down until buttin released or software limit is reached.
 */
public class LauncherScrewTest extends CommandBase {

  private final boolean up_;
  private boolean done = false;

  public LauncherScrewTest(boolean up) {
    addRequirements(robot_.testLauncher_);
    up_ = up;
  }

  @Override
  public void initialize() {
    // robot_.testLauncher_.setScrewPosition01(up_ ? 1. : 0.0);
  }

  @Override
  public void execute() {
    double currentPosition = robot_.testLauncher_.getScrewPosition();
    // SmartDashboard.putNumber("testing?", currentPosition + (up_ ? .1 : -.1));
    robot_.testLauncher_.setScrewPosition(currentPosition + (up_ ? .1 : -.1));
    done = true;
  }

  @Override
  public void end(boolean interrupted) {
    // robot_.testLauncher_.setScrewPosition(robot_.testLauncher_.getScrewPosition()); // stop when button is released
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
