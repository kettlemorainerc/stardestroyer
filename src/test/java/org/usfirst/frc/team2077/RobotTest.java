package org.usfirst.frc.team2077;

public class RobotTest {
	static {
		if(Robot.robot_ == null) new Robot();
	}

	private static boolean autonomous = false;

	public static void forAutonomous() {
		autonomous = true;
	}

	public static void forTeleop() {
		autonomous = false;
	}

	public static void advanceAPeriod() {
		Robot.robot_.robotPeriodic();

		if(autonomous) Robot.robot_.autonomousPeriodic();
		else Robot.robot_.teleopPeriodic();
	}
}
