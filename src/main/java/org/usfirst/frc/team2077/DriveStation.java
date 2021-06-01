/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team2077.commands.*;
import org.usfirst.frc.team2077.subsystems.Crosshairs;

import org.usfirst.frc.team2077.commands.ZeroScrew;


public class DriveStation {
    public final Joystick primaryStick_ = new Joystick(0);
    public final Joystick secondaryStick_ = new Joystick(1);
    public final Joystick testingStick_ = new Joystick(5);
    public final Joystick Flight = new Joystick(2);

    public DriveStation(Subsystem position_,
                        Subsystem target_,
                        Crosshairs crosshairs_) {
        CommandScheduler.getInstance()
                        .setDefaultCommand(position_, new PrimaryStickDrive3Axis());
        CommandScheduler.getInstance()
                        .setDefaultCommand(target_, new TrackTarget());
        CommandScheduler.getInstance()
                        .setDefaultCommand(crosshairs_, new AimCrosshairs(secondaryStick_, testingStick_));

        bindDriverControl(primaryStick_);
        bindTechnicalControl(testingStick_);

        JoystickButton secondary4_ = new JoystickButton(secondaryStick_, 4);
        secondary4_.whenPressed(new LoadLauncherBack());

        JoystickButton secondary9_ = new JoystickButton(secondaryStick_, 9);
        secondary9_.whenPressed(new ZeroScrew());

        JoystickButton secondary10_ = new JoystickButton(secondaryStick_, 10);
        secondary10_.whileHeld(new LauncherScrewTest(false));

        JoystickButton secondary11_ = new JoystickButton(secondaryStick_, 11);
        secondary11_.whileHeld(new LauncherScrewTest(true));
    }

    private static void bindDriverControl(Joystick primary) {
        JoystickButton primaryTrigger = new JoystickButton(primary, 1);
        primaryTrigger.whileHeld(new RunGrabber(1));

        new JoystickButton(primary, 4).whenPressed(new ResetCrosshairs());
    }

    private static void bindTechnicalControl(Joystick testing) {
        new JoystickButton(testing, 1).whenPressed(new ToggleLauncher());
        new JoystickButton(testing, 3).whileHeld(new Launch());
        new JoystickButton(testing, 4).whileHeld(new LoadLauncher());
        new JoystickButton(testing, 10).whileHeld(new SteerToCrosshairs());
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
