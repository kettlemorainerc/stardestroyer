/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;


public class RunGrabber extends CommandBase {

  private double speed_;

  public RunGrabber(double speed) {
    addRequirements(robot_.tgrabber_);
    speed_ = speed;
  }

  @Override
  public void initialize() {
    robot_.tgrabber_.toggleGrabber(speed_);
    //System.out.println("Grabbing!");
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    robot_.tgrabber_.stopGrabber();
    //System.out.println("Stopping!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
