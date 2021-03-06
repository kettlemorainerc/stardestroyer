/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DoNothing extends CommandBase {

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
  

  public DoNothing() {
    System.out.println("################################# DoNothing()");
  }

  @Override
  public void initialize() {
    System.out.println("################################# initialize()");
  }

  @Override
  public void execute() {
    System.out.println("################################# execute()");
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("################################# end(" + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
