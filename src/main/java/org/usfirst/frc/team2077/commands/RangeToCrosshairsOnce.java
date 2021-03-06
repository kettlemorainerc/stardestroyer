/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

public class RangeToCrosshairsOnce extends CommandBase {
  
  public RangeToCrosshairsOnce() {
    this(robot_.constants_.UPPER_TARGET_HEIGHT);
  }

  public RangeToCrosshairsOnce(double targetHeight) {
    addRequirements(robot_.launcher_);
  }

  @Override
  public void initialize() {
    double range = robot_.crosshairs_.getRange();
    boolean ok = robot_.launcher_.setRangeUpper(range);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
