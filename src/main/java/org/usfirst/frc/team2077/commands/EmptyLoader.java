/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import static org.usfirst.frc.team2077.Robot.robot_;


/**
 * Full speed translation for a fixed time.
 */
public class EmptyLoader extends WaitCommand {


  public EmptyLoader() {
    super(3);
    addRequirements(robot_.tgrabber_);
  }

  @Override
  public void initialize() {
    super.initialize();
    robot_.tgrabber_.emptyTop();
    super.initialize();
    robot_.tgrabber_.emptyBottom();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
