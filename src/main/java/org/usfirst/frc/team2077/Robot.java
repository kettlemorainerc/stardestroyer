/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import org.usfirst.frc.team2077.commands.AutonomousCheck;
import org.usfirst.frc.team2077.commands.LauncherSpinTest;
import org.usfirst.frc.team2077.commands.LoadLauncher;
import org.usfirst.frc.team2077.commands.LoadLauncherBack;
import org.usfirst.frc.team2077.commands.Move2;
import org.usfirst.frc.team2077.commands.Nudge;
import org.usfirst.frc.team2077.commands.PrimaryStickDrive3Axis;
import org.usfirst.frc.team2077.commands.RangeToCrosshairs;
import org.usfirst.frc.team2077.commands.RunGrabber;
import org.usfirst.frc.team2077.commands.SteerToCrosshairs;
import org.usfirst.frc.team2077.commands.ToggleLauncher;
import org.usfirst.frc.team2077.commands.TrackTarget;
import org.usfirst.frc.team2077.commands.AimCrosshairs;
import org.usfirst.frc.team2077.commands.EmptyLoader;
import org.usfirst.frc.team2077.commands.LauncherScrewTest;
import org.usfirst.frc.team2077.drivetrain.DriveChassisIF;
import org.usfirst.frc.team2077.drivetrain.DriveModuleIF;
import org.usfirst.frc.team2077.drivetrain.MecanumChassis;
import org.usfirst.frc.team2077.drivetrain.SparkNeoDriveModule;
import org.usfirst.frc.team2077.sensors.AnalogSettings;
import org.usfirst.frc.team2077.sensors.AngleSensor;
import org.usfirst.frc.team2077.sensors.InfraredSensor;
import org.usfirst.frc.team2077.sensors.MicroSwitch;
import org.usfirst.frc.team2077.sensors.PotentialSensor;
import org.usfirst.frc.team2077.subsystems.Crosshairs;
import org.usfirst.frc.team2077.subsystems.Launcher;
import org.usfirst.frc.team2077.subsystems.LauncherIF;
import org.usfirst.frc.team2077.subsystems.Telemetry;
import org.usfirst.frc.team2077.subsystems.TestGrabber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  // Using a constructed constants object instead of statics.
  // Put globally accessible system constants in the Constants class.
  // Other code can access them through Robot.robot_.constants_.<FIELD_NAME>.
  public Constants constants_ = new Constants();


  // Other globally accessible objects...

  // Drive station controls.
  public DriveStation driveStation_;

  // Inter-process data exchange.
  public NetworkTableInstance networkTableInstance_ = NetworkTableInstance.getDefault();

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
  public Launcher testLauncher_; // low-level control for testing

  
  //public TestLauncher tLauncher_; // Bringing back support for the TestLauncher Class though the old instance name

  public Telemetry telemetry_;

  public TestGrabber tgrabber_;


  // Default commands
  //    Autonomous selected via drive station dashboard.
  protected Command autonomous_;
  //    Default teleop robot drive.
  protected Command drive_;
  //    Continuous update of target range and direction based on robot motion.
  protected Command track_;
  //    Operator input of target position relative to robot.
  protected Command aim_;
  //    Continuous update of launcher elevation for target range.
  protected Command range_;
  //     Keep Launcher aimed on target from crosshairs
  protected Command launch_;


  public Subsystem testConfig_;//AJ New



  // Everything "global" hangs off the single instance of Robot,
  // either directly or under one of the above public members.
  public static Robot robot_ = null;
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

    // Container for remote control software objects.
    driveStation_ = new DriveStation();

    // Communications with other processes on the DS.
    networkTableInstance_ = NetworkTableInstance.getDefault();

    // Sensors.

    angleSensor_ = new AngleSensor();

    //analogSettings_ = new AnalogSettings(1, 2, 3);

    potentialSensor_ = new PotentialSensor();

    infraredSensor_ = new InfraredSensor(2, constants_.INFRARED_MAX_LOADED, constants_.INFRARED_MIN_LOADED);
    microSwitch_ = new MicroSwitch(2);

    //----------------------------- DRIVETRAIN -----------------------------//
    DriveModuleIF[] starDestroyerDriveModules = {
      new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.FRONT_RIGHT),  // northeast (right front)
      new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.BACK_RIGHT),  // southeast (right rear)
      new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.BACK_LEFT),  // southwest (left rear)
      new SparkNeoDriveModule(SparkNeoDriveModule.DrivePosition.FRONT_LEFT)   // northwest (left front)
    };

    chassis_ = new MecanumChassis(starDestroyerDriveModules, constants_.STARDESTROYER_WHEELBASE, constants_.STARDESTROYER_TRACK_WIDTH, constants_.STARDESTROYER_WHEEL_RADIUS); 
    
    //----------------------------- SUBSYSTEMS -----------------------------//
    //   These dummy subsystems support sepearable command ownership of robot motion and rotation.
    position_ = new SubsystemBase() {};
    heading_ = new SubsystemBase() {};
    target_ = new SubsystemBase() {};

    telemetry_ = new Telemetry();
        
    //   Ball launcher.
    launcher_ = new Launcher();
    testLauncher_ = launcher_ instanceof Launcher ? (Launcher)launcher_ : null;

    //(new ResetTarPos()).initialize();
    tgrabber_ = new TestGrabber();

    //   Manages an operator-controlled aiming point displayed on DS and used for targeting.    
    crosshairs_ = new Crosshairs(); 


    //----------------------------- INITIAL ROBOT AND TARGET POSITION -----------------------------//
    robot_.chassis_.setPosition(-180, 0, 0); // TODO: Initialize from Smart Dashboard
    double[] p = robot_.chassis_.getPosition();
    robot_.crosshairs_.set(Math.atan2(-p[1], -p[0]), Math.sqrt(p[0]*p[0] + p[1]*p[1]));
    
    System.out.println("CROSSHAIRS:" + crosshairs_);

    SmartDashboard.putBoolean("Run Autonomous", false);

    SmartDashboard.putBoolean("Galactic Search", false);
    SmartDashboard.putBoolean("A", false);
    SmartDashboard.putBoolean("Red", false);

    SmartDashboard.putBoolean("Barrel Racing", false);
    SmartDashboard.putBoolean("Slalom", false);
    SmartDashboard.putBoolean("Bounce", false);

    SmartDashboard.putString("errors", "value");

    //----------------------------- DEFAULT TELEOP COMMANDS -----------------------------//
    //drive_ = new PrimaryStickDrive2Axis();
    drive_ = new PrimaryStickDrive3Axis();
    aim_ = new AimCrosshairs();
    track_ = new TrackTarget();
    // range_ = new RangeToCrosshairs(constants_.UPPER_TARGET_HEIGHT - constants_.FISHEYE_CAMERA_HEIGHT);


    CommandScheduler.getInstance().setDefaultCommand(position_, drive_);
    // Uncomment the following to move rotation to secondary stick.
    //CommandScheduler.getInstance().setDefaultCommand(heading_, new SecondaryStickDrive());
    CommandScheduler.getInstance().setDefaultCommand(target_, track_);
    CommandScheduler.getInstance().setDefaultCommand(crosshairs_, aim_);
    // CommandScheduler.getInstance().setDefaultCommand(launcher_, range_);


    driveStation_.primaryTrigger_.whileHeld(new RunGrabber(0.3));


    driveStation_.secondary2_.whileHeld(new SteerToCrosshairs());
    //driveStation_.secondary3_.whenPressed(new RangeToCrosshairs(constants_.UPPER_TARGET_HEIGHT - constants_.DOUBLE_CAMERA_HEIGHT));
    //driveStation_.secondary3_.whenPressed(new SetAngleTest());
    driveStation_.secondary4_.whenPressed(new LoadLauncherBack());
    driveStation_.secondary5_.whileHeld(new LoadLauncher());
    driveStation_.secondary6_.whenPressed(new LauncherSpinTest(200));
    driveStation_.secondary7_.whenPressed(new LauncherSpinTest(-200));
    //driveStation_.secondary8_.whileHeld(new LauncherSpinTest());
    driveStation_.secondary8_.whenPressed(new LauncherScrewTest(true));//launcher_.getScrewPosition() - 0.1)));
    driveStation_.secondary9_.whenPressed(new LauncherScrewTest(false));
    // driveStation_.secondary11_.whileHeld(new LauncherScrewTest(true));
    driveStation_.secondary11_.whileHeld(new LauncherScrewTest(false));

    // driveStation_.secondaryTrigger_.whileHeld(new LoadLauncher());//HERE
    driveStation_.secondaryTrigger_.whenPressed(new ToggleLauncher());
    // driveStation_.secondaryTrigger_.whileHeld(new Launch());
    // driveStation_.secondaryTrigger_.whileHeld(new EmptyLoader());
    //driveStation_.secondaryTrigger_.whileActiveContinuous(new SteerToCrosshairs());
    //driveStation_.secondaryTrigger_.whileHeld(new ContinousAimToTarget3());


    //----------------------------- KEYPAD COMMANDS -----------------------------//
    // driveStation_.testing1_.whenPressed(new ColorOperations());
    // driveStation_.testing1_.whenPressed(new AutonomousOperations());
    // driveStation_.testing2_.whenPressed(new ElevatorOperations());
    // driveStation_.testing3_.whileHeld(new ZPlaceHolderSensors());


    // // Test code.
    // (new JoystickButton(driveStation_.primaryStick_, 3)).whenPressed(new Move2(40, 0, -180));
    // (new JoystickButton(driveStation_.primaryStick_, 4)).whenPressed(new Move2(40, 0, 180));
    // (new JoystickButton(driveStation_.primaryStick_, 5)).whenPressed(new Move2(20, 0, -90));
    // (new JoystickButton(driveStation_.primaryStick_, 6)).whenPressed(new Move2(20, 0, 90));

    // (new POVButton(driveStation_.primaryStick_, 0)).whenPressed(new Nudge(0, .1));
    // (new POVButton(driveStation_.primaryStick_, 90)).whenPressed(new Nudge(90, .15));
    // (new POVButton(driveStation_.primaryStick_, 180)).whenPressed(new Nudge(180, .1));
    // (new POVButton(driveStation_.primaryStick_, 270)).whenPressed(new Nudge(270, .15));

    // (new JoystickButton(driveStation_.primaryStick_, 7)).whenPressed(new Move2(12, -12));
    // (new JoystickButton(driveStation_.primaryStick_, 8)).whenPressed(new Move2(12, 12));
    // (new JoystickButton(driveStation_.primaryStick_, 9)).whenPressed(new Move2(270));
    // (new JoystickButton(driveStation_.primaryStick_, 10)).whenPressed(new Move2(-270));
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
    CommandScheduler.getInstance().run();
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
    CommandScheduler.getInstance().cancelAll();

  }

  /**
   * Called periodically during test.
   */
  @Override
  public void testPeriodic() {
    
  }
}
