package org.usfirst.frc.team2077.commands;

import org.junit.Test;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.WheelPosition;

import java.util.EnumMap;

public class MoveTest extends ChassisTest<MecanumChassis> {
	@Override
	protected MecanumChassis create(EnumMap<WheelPosition, DriveModuleIF> driveModule) {
		return new MecanumChassis(driveModule, TestClock::getAndIncrementSeconds);
	}

	@Override
	public void beforeEachTest() {
		RobotTest.forTeleop();
	}

	@Test
	public void move_forward_8_correctly_updates_chassis() {
		new Move(8, 0, 0).schedule();
		for(int i = 0; i < 20; i++) {
			System.out.println(new ChassisValues(chassis));
			RobotTest.advanceAPeriod();
		}
	}
}
