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

    public final JoystickButton primaryTrigger_ = new JoystickButton(primaryStick_, 1); 
    public final JoystickButton primary2_ = new JoystickButton(primaryStick_, 2);
    public final JoystickButton primary3_ = new JoystickButton(primaryStick_, 3);
    public final JoystickButton primary4_ = new JoystickButton(primaryStick_, 4);
    public final JoystickButton primary5_ = new JoystickButton(primaryStick_, 5);
    public final JoystickButton primary6_ = new JoystickButton(primaryStick_, 6);
    public final JoystickButton primary7_ = new JoystickButton(primaryStick_, 7);
    public final JoystickButton primary8_ = new JoystickButton(primaryStick_, 8);
    public final JoystickButton primary9_ = new JoystickButton(primaryStick_, 9);
    public final JoystickButton primary10_ = new JoystickButton(primaryStick_, 10);
    public final JoystickButton primary11_ = new JoystickButton(primaryStick_, 11);
    public final JoystickButton primary12_ = new JoystickButton(primaryStick_, 12);

    public final JoystickButton secondaryTrigger_ = new JoystickButton(secondaryStick_, 1); 
    public final JoystickButton secondary2_ = new JoystickButton(secondaryStick_, 2);
    public final JoystickButton secondary3_ = new JoystickButton(secondaryStick_, 3);
    public final JoystickButton secondary4_ = new JoystickButton(secondaryStick_, 4);
    public final JoystickButton secondary5_ = new JoystickButton(secondaryStick_, 5);
    public final JoystickButton secondary6_ = new JoystickButton(secondaryStick_, 6);
    public final JoystickButton secondary7_ = new JoystickButton(secondaryStick_, 7);
    public final JoystickButton secondary8_ = new JoystickButton(secondaryStick_, 8);
    public final JoystickButton secondary9_ = new JoystickButton(secondaryStick_, 9);
    public final JoystickButton secondary10_ = new JoystickButton(secondaryStick_, 10);
    public final JoystickButton secondary11_ = new JoystickButton(secondaryStick_, 11);
    public final JoystickButton secondary24_ = new JoystickButton(secondaryStick_, 24);//AJ CHANGES (1.4)
   
    public final JoystickButton testing1_ = new JoystickButton(testingStick_, 1);
    public final JoystickButton testing2_ = new JoystickButton(testingStick_, 2);
    public final JoystickButton testing3_ = new JoystickButton(testingStick_, 3);
    public final JoystickButton testing4_ = new JoystickButton(testingStick_, 4);
    public final JoystickButton testing5_ = new JoystickButton(testingStick_, 5);
    public final JoystickButton testing6_ = new JoystickButton(testingStick_, 6);
    public final JoystickButton testing7_ = new JoystickButton(testingStick_, 7);
    public final JoystickButton testing8_ = new JoystickButton(testingStick_, 8);
    public final JoystickButton testing9_ = new JoystickButton(testingStick_, 9);
    public final JoystickButton testing10_ = new JoystickButton(testingStick_, 10);
    public final JoystickButton testing11_ = new JoystickButton(testingStick_, 11);
    public final JoystickButton testing12_ = new JoystickButton(testingStick_, 12);
    public final JoystickButton testing13_ = new JoystickButton(testingStick_, 13);
    public final JoystickButton testing14_ = new JoystickButton(testingStick_, 14);
    public final JoystickButton testing15_ = new JoystickButton(testingStick_, 15);
    public final JoystickButton testing16_ = new JoystickButton(testingStick_, 16);
    public final JoystickButton testing17_ = new JoystickButton(testingStick_, 17);
    public final JoystickButton testing18_ = new JoystickButton(testingStick_, 18);
    public final JoystickButton testing19_ = new JoystickButton(testingStick_, 19);
    public final JoystickButton testing20_ = new JoystickButton(testingStick_, 20);
    public final JoystickButton testing21_ = new JoystickButton(testingStick_, 21);
    public final JoystickButton testing22_ = new JoystickButton(testingStick_, 22);
    public final JoystickButton testing23_ = new JoystickButton(testingStick_, 23);
    public final JoystickButton testing24_ = new JoystickButton(testingStick_, 24);

    //    Default teleop robot drive.
    protected Command drive_;
    //    Continuous update of target range and direction based on robot motion.
    protected Command track_;
    //    Operator input of target position relative to robot.
    protected Command aim_;
    
    public DriveStation(Subsystem position_,
                        Subsystem target_,
                        Crosshairs crosshairs_) {

        drive_ = new PrimaryStickDrive3Axis();
        aim_ = new AimCrosshairs();
        track_ = new TrackTarget();
        // range_ = new RangeToCrosshairs(constants_.UPPER_TARGET_HEIGHT - constants_.FISHEYE_CAMERA_HEIGHT);


        CommandScheduler.getInstance()
                        .setDefaultCommand(position_, drive_);
        // Uncomment the following to move rotation to secondary stick.
        //CommandScheduler.getInstance().setDefaultCommand(heading_, new SecondaryStickDrive());
        CommandScheduler.getInstance()
                        .setDefaultCommand(target_, track_);
        CommandScheduler.getInstance()
                        .setDefaultCommand(crosshairs_, aim_);
        // CommandScheduler.getInstance().setDefaultCommand(launcher_, range_);


        primaryTrigger_.whileHeld(new RunGrabber(0.6));
        primary4_.whenPressed(new resetCrosshairs());
        testing1_.whileHeld(new RunGrabber(0.3)); //for flysky controller

        secondary6_.whileHeld(new RunGrabber(.3));

        secondary2_.whileHeld(new SteerToCrosshairs());
        secondary2_.whileHeld(new RangeToCrosshairs());
        //secondary3_.whenPressed(new RangeToCrosshairs(constants_.UPPER_TARGET_HEIGHT - constants_.DOUBLE_CAMERA_HEIGHT));
        secondary4_.whenPressed(new LoadLauncherBack());
        secondary5_.whileHeld(new launch());
        secondaryTrigger_.whileHeld(new LoadLauncher());
        secondary7_.whenPressed(new LauncherSpinTest(-100));
        secondary6_.whenPressed(new LauncherSpinTest(100));
//        secondary8_.whenPressed(new LauncherSpinTest(-10));
        secondary9_.whenPressed(new ZeroScrew());
        secondary10_.whileHeld(new LauncherScrewTest(false));
        secondary11_.whileHeld(new LauncherScrewTest(true));

        secondary3_.whenPressed(new ToggleLauncher());
        //secondaryTrigger_.whileHeld(new ContinousAimToTarget3());


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
     * @param input
     * @param deadBand
     * @param exponent
     * @return
     */
    public static double adjustInputSensitivity(double input, double deadBand, double exponent) {
        return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * Math.signum(input);
    }
}
