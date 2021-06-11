/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import static org.usfirst.frc.team2077.Robot.robot_;
import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;

import org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection;
import org.usfirst.frc.team2077.math.AccelerationLimits;

import java.util.EnumMap;

/**
 * Full speed translation for a fixed time.
 */
public class Nudge extends WaitCommand {

  private final double north_;
  private final double east_;

  public Nudge(double direction, double seconds) {
    super(seconds);
    addRequirements(robot_.position_);
    north_ = Math.cos(Math.toRadians(direction));
    east_ = Math.sin(Math.toRadians(direction));    
  }

  @Override
  public void initialize() {
    super.initialize();
    EnumMap<VelocityDirection, Double> max = robot_.chassis_.getMaximumVelocity();
    robot_.chassis_.setVelocity(north_*max.get(NORTH), east_ * max.get(EAST), (new AccelerationLimits(.5, .5, robot_.chassis_)));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    robot_.chassis_.setVelocity(0, 0, (new AccelerationLimits(.5, .5, robot_.chassis_)));
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
