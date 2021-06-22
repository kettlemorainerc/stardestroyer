package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.junit.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.math.*;
import org.usfirst.frc.team2077.sensors.AngleSensor;
import org.usfirst.frc.team2077.subsystems.Crosshairs;

import java.util.EnumMap;

import static org.junit.Assert.*;

public abstract class ChassisTest<Chassis extends AbstractChassis> {
	private static TestDriveModule NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST;

	protected Chassis chassis;
	static { // Robot needs to initialized to prevent a NullPointer dereference inside Mecanum/AbstractChassis
		new Robot();
		Robot.robot_.heading_ = new Subsystem() {};
		Robot.robot_.position_ = new Subsystem() {};
		Robot.robot_.crosshairs_ = new Crosshairs();
	}

	protected abstract Chassis create(EnumMap<WheelPosition, DriveModuleIF> driveModule);

	@Before
	public final void beforeEach() {
		TestClock.reset();

		NORTH_EAST = new TestDriveModule(11, WheelPosition.NORTH_EAST);
		SOUTH_EAST = new TestDriveModule(8, WheelPosition.SOUTH_EAST);
		SOUTH_WEST = new TestDriveModule(10, WheelPosition.SOUTH_WEST);
		NORTH_WEST = new TestDriveModule(9, WheelPosition.NORTH_EAST);

		EnumMap<WheelPosition, DriveModuleIF> driveModule = new EnumMap<>(WheelPosition.class);
		driveModule.put(WheelPosition.NORTH_EAST, NORTH_EAST);
		driveModule.put(WheelPosition.SOUTH_EAST, SOUTH_EAST);
		driveModule.put(WheelPosition.SOUTH_WEST, SOUTH_WEST);
		driveModule.put(WheelPosition.NORTH_WEST, NORTH_WEST);

		chassis = create(driveModule);
		Robot.robot_.chassis_ = chassis;

		chassis.setGLimits(1 / AccelerationLimits.G, 1 / AccelerationLimits.G);

		beforeEachTest();
	}

	public void beforeEachTest() {}

	protected  <T extends Enum<T>> void assertEnumMapEquals(String message, EnumMap<T, Double> expectedMap, EnumMap<T, Double> actualMap, double delta) {
		double[] expected = new double[expectedMap.size()], actual = new double[expectedMap.size()];

		for(T key : expectedMap.keySet()) {
			expected[key.ordinal()] = expectedMap.get(key);
			actual[key.ordinal()] = actualMap.get(key);
		}

		Assert.assertArrayEquals(message, expected, actual, delta);
	}

	protected void assertPeriodicUpdate(ChassisValues expected) {
		RobotTest.advanceAPeriod();

		ChassisValues actual = new ChassisValues(chassis);
		double delta = 0.000000000000001; // HIGH degree of accuracy
		assertEnumMapEquals("Wheel Velocities", expected.wheelVelocities, actual.wheelVelocities, delta);

		assertEnumMapEquals("Calculated Velocity", expected.calculateVelocity, actual.calculateVelocity, delta);
		assertEnumMapEquals("Set Velocity", expected.setVelocity, actual.setVelocity, delta);
		assertEnumMapEquals("Measured Velocity", expected.measuredVelocity, actual.measuredVelocity, delta);

		assertArrayEquals("Set Position", expected.setPosition.get(), actual.setPosition.get(), delta);
		assertArrayEquals("Measured Position", expected.measuredPosition.get(), actual.measuredPosition.get(), delta);
	}

	protected static class ChassisValues {
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

		@Override
		public String toString() {
			return String.format(
				"\nVelocities:\n\t" +
				"wheel vels: %s\n\t" +
				"calc vel: %s\n\t" +
				"set vel: %s\n\t" +
				"measured vel: %s\n\t" +
				"set pos: %s\n\t" +
				"measured pos: %s\n",
				/*Arrays.toString*/(wheelVelocities),
				/*Arrays.toString*/(calculateVelocity),
				/*Arrays.toString*/(setVelocity),
				/*Arrays.toString*/(measuredVelocity),
				setPosition,
				measuredPosition
			);
		}
	}
}
