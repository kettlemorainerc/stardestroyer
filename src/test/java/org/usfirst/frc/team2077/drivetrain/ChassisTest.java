package org.usfirst.frc.team2077.drivetrain;

import org.junit.Test;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.math.Position;

import java.util.*;

import static org.junit.Assert.*;

public class ChassisTest {
    static {
        new Robot();
        Clock.INCREMENT_ON_CALL = true;
    }
    private static final TestDriveModule NORTH_EAST = new TestDriveModule(11),
        SOUTH_EAST = new TestDriveModule(8),
        SOUTH_WEST = new TestDriveModule(10),
        NORTH_WEST = new TestDriveModule(9);

    private static final MecanumChassis chassis = new MecanumChassis(
        new DriveModuleIF[]{
            NORTH_EAST,
            SOUTH_EAST,
            SOUTH_WEST,
            NORTH_WEST
        },
        new Constants(),
        TestClock::getAndIncrementSeconds
    );

    private void assertPeriodicUpdate(BotValues botValues) {
        chassis.periodic();

        assertEquals(botValues, new BotValues(chassis));
    }

    @Test
    public void max_velocity_is_smallest_drive_module_max_velocity() {
        assertEquals(chassis.getMaximumVelocity()[0], 8, 0);
        assertEquals(chassis.getMaximumVelocity()[1], 8, 0);
        assertEquals(
            Math.abs(new MecanumMath(20.375, 22.625, 4.0, 4.0, 1, 180 / Math.PI).forward(new double[]{8, 8, -8, -8})[2]),
            chassis.getMaximumVelocity()[2],
            0
        );
    }

    @Test
    public void maintains_zero_with_no_input() {
        System.out.println(chassis);
        assertPeriodicUpdate(
            new BotValues().wheelVelocities(0, 0, 0, 0)
                           .measuredVelocities(0, 0, 0)
                           .setVelocities(0, 0, 0)
                           .calculatedVelocities(0, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new BotValues().wheelVelocities(0, 0, 0, 0)
                           .measuredVelocities(0, 0, 0)
                           .setVelocities(0, 0, 0)
                           .calculatedVelocities(0, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new BotValues().wheelVelocities(0, 0, 0, 0)
                           .measuredVelocities(0, 0, 0)
                           .setVelocities(0, 0, 0)
                           .calculatedVelocities(0, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new BotValues().wheelVelocities(0, 0, 0, 0)
                           .measuredVelocities(0, 0, 0)
                           .setVelocities(0, 0, 0)
                           .calculatedVelocities(0, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
        );
        assertPeriodicUpdate(
            new BotValues().wheelVelocities(0, 0, 0, 0)
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
            new BotValues().wheelVelocities(0, 0, 0, 0)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(0, 0, 0)
                           .measuredVelocities(0, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(1, 1, 1, 1)
                           .calculatedVelocities(7.721800000000001, 0, 0)
                           .setVelocities(3.8609000000000004, 0, 0)
                           .measuredVelocities(1, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(2, 2, 2, 2)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(7.721800000000001, 0, 0)
                           .measuredVelocities(2, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(3, 3, 3, 3)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(3, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(4, 4, 4, 4)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(4, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(5, 5, 5, 5)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(5, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(6, 6, 6, 6)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(6, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(7, 7, 7, 7)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(7, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(8, 8, 8, 8)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(8, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(8, 8, 8, 8)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(8, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
       assertPeriodicUpdate(
            new BotValues().wheelVelocities(8, 8, 8, 8)
                           .calculatedVelocities(8, 0, 0)
                           .setVelocities(8, 0, 0)
                           .measuredVelocities(8, 0, 0)
                           .measuredPosition(0, 0, 0)
                           .setPosition(0, 0, 0)
       );
    }


    private static class BotValues {
        // Unset things are considered equal
        double[] wheelVelocities, calculateVelocity, setVelocity, measuredVelocity;
        Position setPosition, measuredPosition;

        BotValues() {}

        BotValues(MecanumChassis chassis) {
            calculateVelocity = chassis.getVelocityCalculated();
            setVelocity = chassis.velocitySet_;
            measuredVelocity = chassis.velocityMeasured_;
            wheelVelocities(
                NORTH_EAST.safeGetVelocity(),
                SOUTH_EAST.safeGetVelocity(),
                SOUTH_WEST.safeGetVelocity(),
                NORTH_WEST.safeGetVelocity()
            );
            setPosition = chassis.positionSet_;
            measuredPosition = chassis.positionMeasured_;
        }

        public BotValues wheelVelocities(double northEast, double southEast, double southWest, double northWest) {
            wheelVelocities = new double[] {northEast, southEast, southWest, northWest};
            return this;
        }

        public BotValues calculatedVelocities(double north, double east, double rotation) {
            calculateVelocity = new double[] {north, east, rotation};
            return this;
        }

        public BotValues setVelocities(double north, double east, double rotation) {
            setVelocity = new double[] {north, east, rotation};
            return this;
        }

        public BotValues measuredVelocities(double north, double east, double rotation) {
            measuredVelocity = new double[] {north, east, rotation};
            return this;
        }

        public BotValues setPosition(double north, double east, double heading) {
            setPosition = new Position(new double[] {north, east, heading});
            return this;
        }

        public BotValues measuredPosition(double north, double east, double heading) {
            measuredPosition = new Position(new double[] {north, east, heading});
            return this;
        }

        @Override
        public boolean equals(Object o) {
            if(this == o) return true;
            if(o == null || getClass() != o.getClass()) return false;
            BotValues botValues = (BotValues) o;
            return (wheelVelocities == null || botValues.wheelVelocities == null || Arrays.equals(wheelVelocities, botValues.wheelVelocities)) &&
                   (calculateVelocity == null || botValues.calculateVelocity == null || Arrays.equals(calculateVelocity, botValues.calculateVelocity)) &&
                   (setVelocity == null || botValues.setVelocity == null || Arrays.equals(setVelocity, botValues.setVelocity)) &&
                   (measuredVelocity == null || botValues.measuredVelocity == null || Arrays.equals(measuredVelocity, botValues.measuredVelocity)) &&
                   (setPosition == null || botValues.setPosition == null || Objects.equals(setPosition, botValues.setPosition)) &&
                   (measuredPosition == null || botValues.measuredPosition == null || Objects.equals(measuredPosition, botValues.measuredPosition));
        }

        @Override
        public int hashCode() {
            int result = Objects.hash(setPosition, measuredPosition);
            result = 31 * result + Arrays.hashCode(wheelVelocities);
            result = 31 * result + Arrays.hashCode(calculateVelocity);
            result = 31 * result + Arrays.hashCode(setVelocity);
            result = 31 * result + Arrays.hashCode(measuredVelocity);
            return result;
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
              Arrays.toString(wheelVelocities),
              Arrays.toString(calculateVelocity),
              Arrays.toString(setVelocity),
              Arrays.toString(measuredVelocity),
              setPosition,
              measuredPosition
            );
        }
    }
}
