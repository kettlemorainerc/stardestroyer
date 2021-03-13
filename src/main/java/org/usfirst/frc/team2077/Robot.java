/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.commands.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.sensors.*;
import org.usfirst.frc.team2077.subsystems.*;

// AJ- Changes
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// AJ- Changes

// The default Command template from WPI splits a lot of top level declaration
// and initialization into a separate RobotContainer class. The main purpose of
// doing so seems to be to make the code less convenient to read :)
// In this example such code has been merged back into this file. - RAB

// The following header comment from the template file is just plain wrong.
// It applies to frc.robot.Main. Probably a cut/paste error as code evolved. - RAB

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // Everything "global" hangs off the single instance of Robot,
    // either directly or under one of the above public members.
    public static Robot robot_ = null;


    // Other globally accessible objects...
    // Using a constructed constants object instead of statics.
    // Put globally accessible system constants in the Constants class.
    // Other code can access them through Robot.robot_.constants_.<FIELD_NAME>.
    public Constants constants_ = new Constants();
    // Drive station controls.
    public DriveStation driveStation_;
    // Inter-process data exchange.
    public NetworkTableInstance networkTableInstance_;
    // Sensors.
    public AngleSensor angleSensor_;
    public AnalogSettings analogSettings_;
    public PotentialSensor potentialSensor_;
    public InfraredSensor infraredSensor_;
    public MicroSwitch microSwitch_;
    // Drive train, including:
    //   Controller/motor/wheel/encoder units for each wheel.
    //   Logic for applying robot level functionality to individual wheels.
    public DriveChassisIF chassis_;
    // Subsystems
    //    The position/heading subsystems operate as flags to allow control
    //    of chassis rotation to move between commands independently of positioning.
    public Subsystem position_;
    public Subsystem heading_;
    public Subsystem target_;
    //    Aiming system for elevating ball launcher and pointing the robot. Displayed on DS video.
    public Crosshairs crosshairs_;
    //    Ball launcher with ajustable elevation and speed based on range to target.
    public LauncherIF launcher_;


    //public TestLauncher tLauncher_; // Bringing back support for the TestLauncher Class though the old instance name
    public Launcher testLauncher_; // low-level control for testing
    public Telemetry telemetry_;
    public TestGrabber tgrabber_;
    public Subsystem testConfig_;//AJ New
    // Default commands
    //    Autonomous selected via drive station dashboard.
    protected Command autonomous_;
    //    Continuous update of launcher elevation for target range.
    protected Command range_;
    //     Keep Launcher aimed on target from crosshairs
    protected Command launch_;

    // This class will be instantiated exactly once, via frc.robot.Main.
    // The constructor initializes the globally accessible static instance,
    // all other initialization happens in robotInit().
    public Robot() {
        robot_ = this;
    }

    /**
     * Run once on startup.
     */
    @Override
    public void robotInit() {
        networkTableInstance_ = NetworkTableInstance.getDefault();
        angleSensor_ = new AngleSensor();
        potentialSensor_ = new PotentialSensor();

        //analogSettings_ = new AnalogSettings(1, 2, 3);


        infraredSensor_ = new InfraredSensor(2, constants_.INFRARED_MAX_LOADED, constants_.INFRARED_MIN_LOADED);
        microSwitch_ = new MicroSwitch(2);


        setupDriveTrain();

        robot_.chassis_.setPosition(-180, 0, 0); // TODO: Initialize from Smart Dashboard
        double[] p = robot_.chassis_.getPosition();
        robot_.crosshairs_.set(Math.atan2(-p[1], -p[0]), Math.sqrt(p[0] * p[0] + p[1] * p[1]));

        System.out.println("CROSSHAIRS:" + crosshairs_);

        crosshairs_ = new Crosshairs();
        //   These dummy subsystems support separate command ownership of robot motion and rotation.
        position_ = new SubsystemBase() {};
        target_ = new SubsystemBase() {};

        // Container for remote control software objects.
        driveStation_ = new DriveStation(position_, target_, crosshairs_);
    }

    public void setupDriveTrain() {
        chassis_ = new MecanumChassis(constants_);
        heading_ = new SubsystemBase() {};


        telemetry_ = new Telemetry();

        launcher_ = new Launcher();
        testLauncher_ = launcher_ instanceof Launcher ? (Launcher) launcher_ : null;

        //(new ResetTarPos()).initialize();
        tgrabber_ = new TestGrabber();
    }

    /**
     * Called every robot packet (generally about 50x/second) no matter the mode.
     * Use this for items like diagnostics that you want run during disabled,
     * autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance()
                        .run();
    }

    // The robot and the drive station exchange data packets around 50x/second so long
    // as they are connected and the robot program is running (hasn't crashed or exited).
    // This packet excahange is what keeps the DS related software objects, i.e. Joysticks,
    // in the robot code updated with their position, etc on the the actual DS, and what
    // keeps "Robot Code" indicator on the DS green.
    //
    // Each time a DS packet is received, the underlying WPILIB code calls one or more
    // xxxPeriodic() methods in this class, first a mode-specific one and then robotPeriodic().
    //
    // Each time the robot mode (disabled, autonomous, teleop, test) changes the appropriate
    // xxxInit() method is called. The robotInit() method is called only once at startup.

    /**
     * Called once each time the robot enters disabled mode.
     * Note that in competition the robot may (or may not?) be
     * disabled briefly between autonomous and teleop.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * Called periodically while robot is disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * Called once each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {

        // autonomous_ = new AutonomousOperations();

        // if (autonomous_ != null) {
        //   autonomous_.schedule();
        // }
        autonomous_ = new AutonomousCheck();
        autonomous_.schedule();
    }

    /**
     * Called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Called once each time the robot enters teleop (operator controlled) mode.
     */
    @Override
    public void teleopInit() {

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // if (autonomous_ != null) {
        // autonomous_.cancel();
        // }
    }

    /**
     * Called periodically during teleop.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * Called once each time the robot enters test mode.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance()
                        .cancelAll();

    }

    /**
     * Called periodically during test.
     */
    @Override
    public void testPeriodic() {

    }
}
