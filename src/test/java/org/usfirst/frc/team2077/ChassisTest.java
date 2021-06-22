package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj2.command.*;
import org.junit.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.drivetrain.SparkNeoDriveModule.*;
import org.usfirst.frc.team2077.math.*;
import org.usfirst.frc.team2077.sensors.AngleSensor;
import org.usfirst.frc.team2077.subsystems.Crosshairs;

import java.util.*;

import static org.junit.Assert.*;

public abstract class ChassisTest<Chassis extends AbstractChassis> {
	private static final TestDriveModule
		NORTH_EAST = new TestDriveModule(11, WheelPosition.NORTH_EAST),
		SOUTH_EAST = new TestDriveModule(8, WheelPosition.SOUTH_EAST),
		SOUTH_WEST = new TestDriveModule(10, WheelPosition.SOUTH_WEST),
		NORTH_WEST = new TestDriveModule(9, WheelPosition.NORTH_EAST);

	protected static AbstractChassis chassis;

	protected ChassisTest() {
		if(Robot.robot_ == null) {
			new Robot();
			Robot.robot_.heading_ = new Subsystem() {};
			Robot.robot_.position_ = new Subsystem() {};
			Robot.robot_.crosshairs_ = new Crosshairs();
		}
		EnumMap<WheelPosition, DriveModuleIF> driveModule = new EnumMap<>(WheelPosition.class);
		driveModule.put(WheelPosition.NORTH_EAST, NORTH_EAST);
		driveModule.put(WheelPosition.SOUTH_EAST, SOUTH_EAST);
		driveModule.put(WheelPosition.SOUTH_WEST, SOUTH_WEST);
		driveModule.put(WheelPosition.NORTH_WEST, NORTH_WEST);

		AbstractChassis chassis = create(driveModule);
		AbstractChassis unregister = ChassisTest.chassis;
		ChassisTest.chassis = chassis;
		Robot.robot_.chassis_ = chassis;
		if(unregister != null) CommandScheduler.getInstance()
		                                       .unregisterSubsystem(unregister);
	}

	protected abstract Chassis create(EnumMap<WheelPosition, DriveModuleIF> driveModule);

	@Before
	public final void beforeEach() {
		TestClock.reset();
//		Robot.robot_.chassis_ = chassis;

		Robot.robot_.chassis_.setGLimits(1 / AccelerationLimits.G, 1 / AccelerationLimits.G);

		beforeEachTest();
	}

	public void beforeEachTest() {}

	private static final double EXPECTED_DELTA = 0.000000002;
	protected static <T extends Enum<T>> void assertEnumMapEquals(String message, EnumMap<T, Double> expectedMap, EnumMap<T, Double> actualMap) {
		for(T key : expectedMap.keySet()) {
			assertEquals(key + " " + message, expectedMap.get(key), actualMap.get(key), EXPECTED_DELTA);
		}
	}

	protected static void assertPositionsEqual(String message, double[] expected, double[] actual) {
		for(VelocityDirection dir : VelocityDirection.values()) {
			int i = dir.ordinal();
			assertEquals(dir + " " + message, expected[i], actual[i], EXPECTED_DELTA);
		}
	}

	private static void assertChassisValues(ChassisValues expected, ChassisValues actual) {
		assertEnumMapEquals("Wheel Velocity", expected.wheelVelocities, actual.wheelVelocities);

		assertEnumMapEquals("Calculated Velocity", expected.calculateVelocity, actual.calculateVelocity);
		assertEnumMapEquals("Set Velocity", expected.setVelocity, actual.setVelocity);
		assertEnumMapEquals("Measured Velocity", expected.measuredVelocity, actual.measuredVelocity);

		assertArrayEquals("Set Position", expected.setPosition.get(), actual.setPosition.get(), EXPECTED_DELTA);
		assertArrayEquals("Measured Position", expected.measuredPosition.get(), actual.measuredPosition.get(), EXPECTED_DELTA);
	}

	public static void assertPeriodicUpdate(ChassisValues expected) {
		RobotTest.advanceAPeriod();

		ChassisValues actual = new ChassisValues(chassis);
		assertChassisValues(expected, actual);
		System.out.println(Robot.robot_.chassis_.getAccelerationLimits());
	}

	public static class ChassisValues {
		// Unset things are considered equal
		EnumMap<VelocityDirection, Double> calculateVelocity, setVelocity, measuredVelocity;
		EnumMap<WheelPosition, Double> wheelVelocities;
		Position setPosition, measuredPosition;

		public ChassisValues() {}

		public ChassisValues(AbstractChassis chassis) {
			calculateVelocity = chassis.getVelocityCalculated();
			setVelocity = chassis.getVelocitySet();
			measuredVelocity = chassis.getVelocityMeasured();
			wheelVelocities(
				NORTH_EAST.getVelocity(),
				SOUTH_EAST.getVelocity(),
				SOUTH_WEST.getVelocity(),
				NORTH_WEST.getVelocity()
			);
			setPosition = chassis.getPosition();
			measuredPosition = chassis.getMeasuredPosition();
		}

		public ChassisValues wheelVelocities(double northEast, double southEast, double southWest, double northWest) {
			wheelVelocities = MecanumMathTest.wheelVelocities(northEast, southEast, southWest, northWest);
//            wheelVelocities = new double[] {northEast, southEast, southWest, northWest};
			return this;
		}

		public ChassisValues calculatedVelocities(double north, double east, double rotation) {
//            calculateVelocity = new double[] {north, east, rotation};
			calculateVelocity = MecanumMathTest.botVelocity(north, east, rotation);
			return this;
		}

		public ChassisValues setVelocities(double north, double east, double rotation) {
//            setVelocity = new double[] {north, east, rotation};
			setVelocity = MecanumMathTest.botVelocity(north, east, rotation);
			return this;
		}

		public ChassisValues measuredVelocities(double north, double east, double rotation) {
//            measuredVelocity = new double[] {north, east, rotation};
			measuredVelocity = MecanumMathTest.botVelocity(north, east, rotation);
			return this;
		}

		public ChassisValues setPosition(double north, double east, double heading) {
			setPosition = new Position(new double[] {north, east, heading});
			return this;
		}

		public ChassisValues measuredPosition(double north, double east, double heading) {
			measuredPosition = new Position(new double[] {north, east, heading});
			return this;
		}

		private String joining(EnumMap<?, ?> map) {
			return String.join(
				", ",
				map.values()
				   .stream()
				   .map(Objects::toString)
				   .toArray(String[]::new)
			);
		}

		private String assertionLine(String method, EnumMap<?, ?> map) {
			return "                       ." + method + "(" + joining(map) + ")\n";
		}

		private String asAssertion() {
			return "assertPeriodicUpdate(\n" +
			       "    new ChassisValues().wheelVelocities(" + joining(wheelVelocities) + ")\n" +
			       assertionLine("calculatedVelocities", calculateVelocity) +
			       assertionLine("setVelocities", setVelocity) +
			       assertionLine("measuredVelocities", measuredVelocity) +
			       assertionLine("setPosition", setPosition) +
			       assertionLine("measuredPosition", measuredPosition) +
			       ");\n";
		}

		@Override
		public String toString() {
			return asAssertion();
//			StringBuilder wheels = new StringBuilder("[Wheels | "),
//				calculatedVelocity = new StringBuilder("[Calc Vel | "),
//				setVel = new StringBuilder("[Set Vel | "),
//				measuredVel = new StringBuilder("[Measured Vel | "),
//				setPos = new StringBuilder("[Set Pos | "),
//				measuredPos = new StringBuilder("[Measured Pos | ");
//
//			for(WheelPosition pos : WheelPosition.values()) {
//				String[] positionParts = pos.name().split("_");
//				wheels.append(positionParts[0].charAt(0))
//				      .append(positionParts[1].charAt(0))
//				      .append("{")
//				      .append(wheelVelocities.get(pos))
//				      .append("} ");
//			}
//
//			for(VelocityDirection dir : VelocityDirection.values()) {
//				char dirChar = dir.name().charAt(0);
//
//				calculatedVelocity.append(dirChar)
//				                  .append(": ")
//				                  .append('{')
//				                  .append(calculateVelocity.get(dir))
//				                  .append("} ");
//
//				setVel.append(dirChar)
//				                  .append(": ")
//				                  .append('{')
//				                  .append(setVelocity.get(dir))
//				                  .append("} ");
//
//				measuredVel.append(dirChar)
//				                  .append(": ")
//				                  .append('{')
//				                  .append(measuredVelocity.get(dir))
//				                  .append("} ");
//
//				setPos.append(dirChar)
//				                  .append(": ")
//				                  .append('{')
//				                  .append(setPosition.get(dir))
//				                  .append("} ");
//
//				measuredPos.append(dirChar)
//				                  .append(": ")
//				                  .append('{')
//				                  .append(measuredPosition.get(dir))
//				                  .append("} ");
//			}
//
//			return "Chassis Velocities: " +
//			       wheels.toString().replaceFirst(" $", "]") +
//			       calculatedVelocity.toString().replaceFirst(" $", "]") +
//			       setVel.toString().replaceFirst(" $", "]") +
//			       "\n                    " +
//			       measuredVel.toString().replaceFirst(" $", "]") +
//			       setPos.toString().replaceFirst(" $", "]") +
//			       measuredPos.toString().replaceFirst(" $", "]");
		}
	}
}
