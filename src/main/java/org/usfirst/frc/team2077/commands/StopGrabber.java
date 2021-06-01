/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.robot_;


public class StopGrabber extends CommandBase {

  private double speed_;

  public StopGrabber() {
    addRequirements(robot_.tgrabber_);
  }

  @Override
  public void initialize() {
    robot_.tgrabber_.stopGrabber();
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
