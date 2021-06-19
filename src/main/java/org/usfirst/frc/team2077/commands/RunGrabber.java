/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static org.usfirst.frc.team2077.Robot.*;


public class RunGrabber extends CommandBase {

  private double speed_;
  private final double activationPoint_ = .75;
  private final Joystick flight;
  private final JoystickButton activate;

  public RunGrabber(Joystick flight, JoystickButton activate) {
    addRequirements(robot_.tgrabber_);

    this.flight = flight;
    this.activate = activate;
  }

  @Override
  public void initialize() {
//    robot_.tgrabber_.toggleGrabber(speed_);
    //System.out.println("Grabbing!");
  }

  @Override
  public void execute() {
    double z = this.flight.getZ();
    boolean button = this.activate.get();

    if (button || z > activationPoint_) {
      robot_.tgrabber_.toggleGrabber(speed_);
    } else {
      robot_.tgrabber_.stopGrabber();
    }

  }

  @Override
  public void end(boolean interrupted) {
    robot_.tgrabber_.stopGrabber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
