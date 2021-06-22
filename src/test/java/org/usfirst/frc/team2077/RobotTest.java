package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj2.command.*;

import java.util.*;

public class RobotTest {

	private static boolean autonomous = false;
	private static List<Command> commands = new LinkedList<>();

	public static void forAutonomous() {
		autonomous = true;
		// I'm not doing this for Teleop since that would bind a bunch of controllers/buttons we DO NOT want
		Robot.robot_.autonomousInit();
		commands = new LinkedList<>();
	}

	public static void forTeleop() {
		autonomous = false;
		commands = new LinkedList<>();
	}

	public static void includeCommands(Command... commands) {
		RobotTest.commands.addAll(List.of(commands));
		for(Command com : commands) com.initialize();
	}

	public static void advanceAPeriod() {
		if(Robot.robot_ == null) new Robot();
		Robot.robot_.robotPeriodic();

		commands.forEach(command -> {
			if(command.isFinished()) {
				command.end(false);
				commands.remove(command);
			} else command.execute();
		});
		if(autonomous) Robot.robot_.autonomousPeriodic();
		else Robot.robot_.teleopPeriodic();
	}
}
