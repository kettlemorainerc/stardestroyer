/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.commands.*;
import org.usfirst.frc.team2077.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.subsystems.*;
import org.usfirst.frc.team2077.subsystems.controller.*;
import org.usfirst.frc.team2077.subsystems.controller.KeypadController.SupportedAxis;
import org.usfirst.frc.team2077.subsystems.controller.XboxController;

import java.util.*;


public class DriveStation {
    public final ControllerBinding driver = new FlySkyController(2);
//    public final ControllerBinding driver = new FlightStickController(0);
//    public final ControllerBinding driver = new XboxController(3); // ACTUAL Xbox controller

    public final ControllerBinding technical = new KeypadController(5, SupportedAxis.NORTH, SupportedAxis.EAST);
//    public final ControllerBinding technical = new FlightStickController(1);
//    public final ControllerBinding technical = new XboxController(4); // Guitar Hero Controller


    public final List<Subsystem> tracked;
//    public final Joystick primaryStick_ = new Joystick(0);
//    public final Joystick secondaryStick_ = new Joystick(1);
//    public final Joystick testingStick_ = new Joystick(5);
//    public final Joystick Flight = new Joystick(2);

    public DriveStation(AbstractChassis chassis,
                        Subsystem position_,
                        Subsystem target_,
                        Crosshairs crosshairs_,
                        TestGrabber grabber_) {
        CommandScheduler.getInstance()
                        .setDefaultCommand(grabber_, new RunGrabber(driver, technical, grabber_));
        CommandScheduler.getInstance()
                        .setDefaultCommand(position_, new PrimaryStickDrive3Axis(position_, driver));
        CommandScheduler.getInstance()
                        .setDefaultCommand(target_, new TrackTarget(chassis, crosshairs_, target_));
        CommandScheduler.getInstance()
                        .setDefaultCommand(crosshairs_, new AimCrosshairs(technical));

        tracked = List.of(grabber_, position_, target_, crosshairs_);

        driver.bindDriver();
        technical.bindTechnical();
    }

    public void cancel() {
        driver.cancelDriver();
        technical.cancelTechnical();

        tracked.forEach(CommandScheduler.getInstance()::unregisterSubsystem);
    }

    /**
     * Condition control axis input to improve driveability.
     * Each axis has a center dead band in which the output for that axis is always zero.
     * Outside the dead band the output increases exponentially from zero to 1 or -1.
     */
    public static double adjustInputSensitivity(double input, double deadBand, double exponent) {
        return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * Math.signum(input);
    }
}
