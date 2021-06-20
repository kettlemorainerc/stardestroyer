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
import org.usfirst.frc.team2077.subsystems.Crosshairs;

import java.util.*;


public class DriveStation {
    public final Joystick primaryStick_ = new Joystick(0);
    public final Joystick secondaryStick_ = new Joystick(1);
    public final Joystick testingStick_ = new Joystick(5);
    public final Joystick Flight = new Joystick(2);
    
    private final List<Joystick> joysticks = new LinkedList<>();
    private final List<JoystickButton> buttons = new LinkedList<>();
    private final List<Subsystem> subsystems = new LinkedList<>();

    public DriveStation(Subsystem position_,
                        Subsystem target_,
                        Crosshairs crosshairs_,
                        Subsystem grabber_) {
        CommandScheduler.getInstance()
                .setDefaultCommand(grabber_, new RunGrabber(Flight, new JoystickButton(testingStick_, 9)));
        CommandScheduler.getInstance()
                        .setDefaultCommand(position_, new PrimaryStickDrive3Axis());
        CommandScheduler.getInstance()
                        .setDefaultCommand(target_, new TrackTarget());
        CommandScheduler.getInstance()
                        .setDefaultCommand(crosshairs_, new AimCrosshairs(secondaryStick_, testingStick_));

//        bindDriverControl(primaryStick_);
        bindTechnicalControl(testingStick_);
    }

    private static void bindDriverControl(Joystick primary) {
        JoystickButton primaryTrigger = new JoystickButton(primary, 1);
//        primaryTrigger.whileHeld(new RunGrabber());

        new JoystickButton(primary, 4).whenPressed(new ResetCrosshairs());
    }

    private void bindTechnicalControl(Joystick testing) {
        new JoystickButton(testing, 1).whenPressed(new TurnOffLauncher());
        new JoystickButton(testing, 3).whileHeld(new Launch());
        new JoystickButton(testing, 4).whileHeld(new LoadLauncher());
        new JoystickButton(testing, 8).whenPressed(new LoadLauncherBack());
        new JoystickButton(testing, 10).whileHeld(new SteerToCrosshairs());
        new JoystickButton(testing, 11).whileHeld(new CenterBallOnVision());
        new JoystickButton(testing, 12).whileHeld(new LauncherScrewTest(false));
        new JoystickButton(testing, 16).whileHeld(new LauncherScrewTest(true));
    }

    /**
     * Condition control axis input to improve driveability.
     * Each axis has a center dead band in which the output for that axis is always zero.
     * Outside the dead band the output increases exponentially from zero to 1 or -1.
     */
    public static double adjustInputSensitivity(double input, double deadBand, double exponent) {
        return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * Math.signum(input);
    }

    public void cancel() {
//        CommandScheduler.getInstance().unregisterSubsystem(subsystems.toArray(new Subsystem[0]));
//        joysticks.forEach(stick -> {
//            stick.`
//        });
    }
}
