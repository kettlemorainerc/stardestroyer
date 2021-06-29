/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.subsystems.TestGrabber;
import org.usfirst.frc.team2077.subsystems.controller.ControllerBinding;
import org.usfirst.frc.team2077.subsystems.controller.ControllerBinding.Axis;

import java.util.function.*;

import static org.usfirst.frc.team2077.Robot.*;


public class RunGrabber extends CommandBase {

  private static final double TOGGLE_AT = .75;
  private final Function<Axis, Double> driver, technical;
  private final TestGrabber grabber;

  public RunGrabber(ControllerBinding driver, ControllerBinding technical, TestGrabber grabber) {
    addRequirements(grabber);
    this.driver = driver::getAxis;
    this.technical = technical::getAxis;
    this.grabber = grabber;
  }

  @Override
  public void initialize() {
//    grabber.toggleGrabber(speed_);
    //System.out.println("Grabbing!");
  }

  @Override
  public void execute() {
    if (driver.apply(Axis.GRABBER) >= TOGGLE_AT || technical.apply(Axis.GRABBER) >= TOGGLE_AT) {
      grabber.toggleGrabber(0);
    } else {
      grabber.stopGrabber();
    }
  }

  @Override
  public void end(boolean interrupted) {
    grabber.stopGrabber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
