/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class RangeToCrosshairs extends CommandBase {
  
  public RangeToCrosshairs() {
    this(robot_.constants_.UPPER_TARGET_HEIGHT);
  }

  public RangeToCrosshairs(double targetHeight) {
    addRequirements(robot_.launcher_);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double range = robot_.crosshairs_.getRange();
    boolean ok = robot_.launcher_.setRangeUpper(range);
    //System.out.println("RANGE:" + range + (!ok ? "(out of range)" : ""));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
