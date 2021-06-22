package org.usfirst.frc.team2077.drivetrain;

import org.junit.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.math.Position;

import java.util.*;

import static org.junit.Assert.*;
import static org.usfirst.frc.team2077.math.AccelerationLimits.*;

public class ChassisTest {
    private static TestDriveModule NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST;

    private static MecanumChassis chassis;
    static { // Robot needs to initialized to prevent a NullPointer dereference inside Mecanum/AbstractChassis
        new Robot();
    }

    @Before
    public void beforeEach() {
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

        chassis = new MecanumChassis(driveModule, TestClock::getAndIncrementSeconds);

        chassis.setGLimits(1 / G, 1 / G); // set acc/deceleration limits to ~ 1 in/s
    }

    private <T extends Enum<T>> void assertEnumMapEquals(String message, EnumMap<T, Double> expectedMap, EnumMap<T, Double> actualMap, double delta) {
        double[] expected = new double[expectedMap.size()], actual = new double[expectedMap.size()];

        for(T key : expectedMap.keySet()) {
            expected[key.ordinal()] = expectedMap.get(key);
            actual[key.ordinal()] = actualMap.get(key);
        }

        Assert.assertArrayEquals(message, expected, actual, delta);
    }

    private void assertPeriodicUpdate(ChassisValues expected) {
        chassis.periodic();

        ChassisValues actual = new ChassisValues(chassis);
        double delta = 0.000000000000001; // HIGH degree of accuracy
        assertEnumMapEquals("Wheel Velocities", expected.wheelVelocities, actual.wheelVelocities, delta);

        assertEnumMapEquals("Calculated Velocity", expected.calculateVelocity, actual.calculateVelocity, delta);
        assertEnumMapEquals("Set Velocity", expected.setVelocity, actual.setVelocity, delta);
        assertEnumMapEquals("Measured Velocity", expected.measuredVelocity, actual.measuredVelocity, delta);

        assertArrayEquals("Set Position", expected.setPosition.get(), actual.setPosition.get(), delta);
        assertArrayEquals("Measured Position", expected.measuredPosition.get(), actual.measuredPosition.get(), delta);
    }

    @Test
    public void max_velocity_is_smallest_drive_module_max_velocity() {
        assertEquals(8, chassis.getMaximumVelocity().get(VelocityDirection.NORTH), 0);
        assertEquals(8, chassis.getMaximumVelocity().get(VelocityDirection.EAST), 0);
        assertEquals(
            21.319359818821333,
            chassis.getMaximumVelocity().get(VelocityDirection.ROTATION),
            0
        );
    }

    @Test
    public void maintains_zero_with_no_input() {
        System.out.println(chassis);
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0, 0, 0, 0)
                               .measuredVelocities(0, 0, 0)
                               .setVelocities(0, 0, 0)
                               .calculatedVelocities(0, 0, 0)
                               .measuredPosition(0, 0, 0)
                               .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0, 0, 0, 0)
                               .measuredVelocities(0, 0, 0)
                               .setVelocities(0, 0, 0)
                               .calculatedVelocities(0, 0, 0)
                               .measuredPosition(0, 0, 0)
                               .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0, 0, 0, 0)
                               .measuredVelocities(0, 0, 0)
                               .setVelocities(0, 0, 0)
                               .calculatedVelocities(0, 0, 0)
                               .measuredPosition(0, 0, 0)
                               .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0, 0, 0, 0)
                               .measuredVelocities(0, 0, 0)
                               .setVelocities(0, 0, 0)
                               .calculatedVelocities(0, 0, 0)
                               .measuredPosition(0, 0, 0)
                               .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0, 0, 0, 0)
                               .measuredVelocities(0, 0, 0)
                               .setVelocities(0, 0, 0)
                               .calculatedVelocities(0, 0, 0)
                               .measuredPosition(0, 0, 0)
                               .setPosition(0, 0, 0)
        );
    }

    @Test
    public void moves_forward_in_straight_line() {
        chassis.setVelocity(8, 0);
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.2, 0.2, 0.2, 0.2)
                               .calculatedVelocities(0.2, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(0, 0, 0)
                               .setPosition(0, 0, 0)
                               .measuredPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.4, 0.4, 0.4, 0.4)
                               .calculatedVelocities(0.4, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(0.2, 0, 0)
                               .setPosition(0.04, 0, 0)
                               .measuredPosition(0.04, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(.6, .6, .6, .6)
                               .calculatedVelocities(.6, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(.4, 0, 0)
                               .setPosition(0.12, 0, 0)
                               .measuredPosition(0.12, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.8, 0.8, 0.8, 0.8)
                               .calculatedVelocities(.8, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(0.6, 0, 0)
                               .setPosition(.24, 0, 0)
                               .measuredPosition(.24, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1, 1, 1, 1)
                               .calculatedVelocities(1, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(.8, 0, 0)
                               .setPosition(.4, 0, 0)
                               .measuredPosition(.4, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.2, 1.2, 1.2, 1.2)
                               .calculatedVelocities(1.2, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(1, 0, 0)
                               .setPosition(.6, 0, 0)
                               .measuredPosition(.6, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.4, 1.4, 1.4, 1.4)
                               .calculatedVelocities(1.4, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(1.2, 0, 0)
                               .setPosition(.84, 0, 0)
                               .measuredPosition(.84, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.6, 1.6, 1.6, 1.6)
                               .calculatedVelocities(1.6, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(1.4, 0, 0)
                               .setPosition(1.12, 0, 0)
                               .measuredPosition(1.12, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.8, 1.8, 1.8, 1.8)
                               .calculatedVelocities(1.8, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(1.6, 0, 0)
                               .setPosition(01.44, 0, 0)
                               .measuredPosition(1.44, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(2, 2, 2, 2)
                               .calculatedVelocities(2, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(1.8, 0, 0)
                               .setPosition(1.8, 0, 0)
                               .measuredPosition(1.8, 0, 0)
       );
       assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(2.2, 2.2, 2.2, 2.2)
                               .calculatedVelocities(2.2, 0, 0)
                               .setVelocities(8, 0, 0)
                               .measuredVelocities(2, 0, 0)
                               .setPosition(2.2, 0, 0)
                               .measuredPosition(2.2, 0, 0)
       );
    }

    private static class ChassisValues {
        // Unset things are considered equal
        EnumMap<VelocityDirection, Double> calculateVelocity, setVelocity, measuredVelocity;
        EnumMap<WheelPosition, Double> wheelVelocities;
        Position setPosition, measuredPosition;

        ChassisValues() {}

        ChassisValues(MecanumChassis chassis) {
            calculateVelocity = chassis.getVelocityCalculated();
            setVelocity = chassis.getVelocitySet();
            measuredVelocity = chassis.getVelocityMeasured();
            wheelVelocities(
                NORTH_EAST.getVelocity(),
                SOUTH_EAST.getVelocity(),
                SOUTH_WEST.getVelocity(),
                NORTH_WEST.getVelocity()
            );
            setPosition = chassis.positionSet_;
            measuredPosition = chassis.positionMeasured_;
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
