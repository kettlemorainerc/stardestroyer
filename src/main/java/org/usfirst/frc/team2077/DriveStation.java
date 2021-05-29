/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
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
        // Uncomment the following to move rotation to secondary stick.
        //CommandScheduler.getInstance().setDefaultCommand(heading_, new SecondaryStickDrive());
        CommandScheduler.getInstance()
                        .setDefaultCommand(target_, new TrackTarget());
        CommandScheduler.getInstance()
                        .setDefaultCommand(crosshairs_, new AimCrosshairs(secondaryStick_, testingStick_));
        // CommandScheduler.getInstance().setDefaultCommand(launcher_, range_);

        JoystickButton primaryTrigger_ = new JoystickButton(primaryStick_, 1);
        primaryTrigger_.whileHeld(new RunGrabber(1));
        // testing1_.whileHeld(new RunGrabber(0.3)); //for flysky controller

        JoystickButton primary4_ = new JoystickButton(primaryStick_, 4);
        primary4_.whenPressed(new ResetCrosshairs());

        JoystickButton secondary2_ = new JoystickButton(secondaryStick_, 2);
        secondary2_.whileHeld(new SteerToCrosshairs());

        JoystickButton secondary3_ = new JoystickButton(secondaryStick_, 3);
        //secondary3_.whenPressed(new RangeToCrosshairs(constants_.UPPER_TARGET_HEIGHT - constants_.DOUBLE_CAMERA_HEIGHT));
        secondary3_.whenPressed(new ToggleLauncher());

        JoystickButton secondary4_ = new JoystickButton(secondaryStick_, 4);
        secondary4_.whenPressed(new LoadLauncherBack());

        JoystickButton secondary5_ = new JoystickButton(secondaryStick_, 5);
        secondary5_.whileHeld(new Launch());

        JoystickButton testing1_ = new JoystickButton(testingStick_, 1);
        testing1_.whileHeld(new RunGrabber(0.5)); //for flysky controller

        JoystickButton secondary6_ = new JoystickButton(secondaryStick_, 6);
        secondary6_.whenPressed(new LauncherSpinTest(100));
        secondary6_.whileHeld(new RunGrabber(.5));

        JoystickButton secondary7_ = new JoystickButton(secondaryStick_, 7);
        secondary7_.whenPressed(new LauncherSpinTest(-100));


        JoystickButton secondaryTrigger_ = new JoystickButton(secondaryStick_, 1);
        secondaryTrigger_.whileHeld(new LoadLauncher());

        //secondaryTrigger_.whileHeld(new ContinousAimToTarget3());
//        secondary2_.whileHeld(new RangeToCrosshairs());
//        secondary3_.whenPressed(new RangeToCrosshairs(constants_.UPPER_TARGET_HEIGHT - constants_.DOUBLE_CAMERA_HEIGHT));

//        JoystickButton secondary8_ = new JoystickButton(secondaryStick_, 8);
//        secondary8_.whenPressed(new LauncherSpinTest(-10));

        JoystickButton secondary9_ = new JoystickButton(secondaryStick_, 9);
        secondary9_.whenPressed(new ZeroScrew());
//
        JoystickButton secondary10_ = new JoystickButton(secondaryStick_, 10);
        secondary10_.whileHeld(new LauncherScrewTest(false));

        JoystickButton secondary11_ = new JoystickButton(secondaryStick_, 11);
        secondary11_.whileHeld(new LauncherScrewTest(true));


        // testing1_.whenPressed(new ColorOperations());
        // testing1_.whenPressed(new AutonomousOperations());
        // testing2_.whenPressed(new ElevatorOperations());
        // testing3_.whileHeld(new ZPlaceHolderSensors());


        // // Test code.
        // (new JoystickButton(primaryStick_, 3)).whenPressed(new Move(40, 0, -180));
        // (new JoystickButton(primaryStick_, 4)).whenPressed(new Move(40, 0, 180));
        // (new JoystickButton(primaryStick_, 5)).whenPressed(new Move(20, 0, -90));
        // (new JoystickButton(primaryStick_, 6)).whenPressed(new Move(20, 0, 90));

        // (new POVButton(primaryStick_, 0)).whenPressed(new Nudge(0, .1));
        // (new POVButton(primaryStick_, 90)).whenPressed(new Nudge(90, .15));
        // (new POVButton(primaryStick_, 180)).whenPressed(new Nudge(180, .1));
        // (new POVButton(primaryStick_, 270)).whenPressed(new Nudge(270, .15));

        // (new JoystickButton(primaryStick_, 7)).whenPressed(new Move(12, -12));
        // (new JoystickButton(primaryStick_, 8)).whenPressed(new Move(12, 12));
        // (new JoystickButton(primaryStick_, 9)).whenPressed(new Move(270));
        // (new JoystickButton(primaryStick_, 10)).whenPressed(new Move(-270));
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
