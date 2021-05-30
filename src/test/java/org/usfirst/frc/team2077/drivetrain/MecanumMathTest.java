package org.usfirst.frc.team2077.drivetrain;

import org.junit.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

import java.util.*;

import static org.junit.Assert.*;

public class MecanumMathTest {
    private MecanumMath math = new MecanumMath(20.375, 22.625, 4d, 4d, 1, 180 / Math.PI);

    private EnumMap<WheelPosition, Double> wheelVelocities(double northEast, double southEast, double southWest, double northWest) {
        EnumMap<WheelPosition, Double> velocities = new EnumMap<>(WheelPosition.class);
        velocities.put(WheelPosition.NORTH_EAST, northEast);
        velocities.put(WheelPosition.SOUTH_EAST, southEast);
        velocities.put(WheelPosition.SOUTH_WEST, southWest);
        velocities.put(WheelPosition.NORTH_WEST, northWest);
        return velocities;
//        return new double[] {northEast, southEast, southWest, northWest} ;
    }
    
    private EnumMap<VelocityDirection, Double> botVelocity(double north, double east, double rotation) {
        EnumMap<VelocityDirection, Double> velocity = new EnumMap<>(VelocityDirection.class);
        velocity.put(VelocityDirection.NORTH, north);
        velocity.put(VelocityDirection.EAST, east);
        velocity.put(VelocityDirection.ROTATION, rotation);
        
        return velocity;
//        return new double[] {north, east, rotation};
    }

    public <T extends Enum<T>> void assertEnumMapEquals(EnumMap<T, Double> expected, EnumMap<T, Double> actual) {
        for(T key : expected.keySet()) {
            assertEquals(String.format("Value of %s outside of wanted delta", key), expected.get(key), actual.get(key), .1);
        }
    }

    @Test
    public void forward_produces_expected_results() {
        // move directionally N/S/E/W
        
        assertEnumMapEquals(
            botVelocity(4, 0, 0),
            math.forward(wheelVelocities(4, 4, 4, 4))
        );
        assertEnumMapEquals(
            botVelocity(-11, 0, 0),
            math.forward(wheelVelocities(-11, -11, -11, -11))
        );
        assertEnumMapEquals(
            botVelocity(0, 77, 0),
            math.forward(wheelVelocities(-77, 77, -77, 77))
        );
        assertEnumMapEquals(
            botVelocity(0, -45, 0),
            math.forward(wheelVelocities(45, -45, 45, -45))
        );

        // rotation
        assertEnumMapEquals(
            botVelocity(0, 0, -10.65),
            math.forward(wheelVelocities(4, 4, -4, -4))
        );
        assertEnumMapEquals(
            botVelocity(0, 0, 39.97),
            math.forward(wheelVelocities(-15, -15, 15, 15))
        );

        // arbitrary inputs for more coverage
        assertEnumMapEquals(
            botVelocity(72.25, 55.75, -177.88),
            math.forward(wheelVelocities(83.24, 194.74, -50.24, 61.25))
        );
        assertEnumMapEquals(
            botVelocity(44.75, 16.75, -44.63),
            math.forward(wheelVelocities(44.74, 78.24, 11.25, 44.75))
        );
        assertEnumMapEquals(
            botVelocity(1.75, -8.25, 3.33),
            math.forward(wheelVelocities(8.75, -7.74, 11.24, -5.25))
        );
    }
    
    @Test
    public void inverse_produces_expected_results() {
        // move directionally N/S/E/W
        assertEnumMapEquals(
            wheelVelocities(4, 4, 4, 4),
            math.inverse(botVelocity(4, 0, 0))
        );
        assertEnumMapEquals(
            wheelVelocities(-11, -11, -11, -11),
            math.inverse(botVelocity(-11, 0, 0))
        );
        assertEnumMapEquals(
            wheelVelocities(-77, 77, -77, 77),
            math.inverse(botVelocity(0, 77, 0))
        );
        assertEnumMapEquals(
            wheelVelocities(45, -45, 45, -45),
            math.inverse(botVelocity(0, -45, 0))
        );

        // rotation
        assertEnumMapEquals(
            wheelVelocities(4, 4, -4, -4),
            math.inverse(botVelocity(0, 0, -10.65))
        );
        assertEnumMapEquals(
            wheelVelocities(-15, -15, 15, 15),
            math.inverse(botVelocity(0, 0, 39.97))
        );

        // arbitrary inputs for more coverage
        assertEnumMapEquals(
            wheelVelocities(83.24, 194.74, -50.24, 61.25),
            math.inverse(botVelocity(72.25, 55.75, -177.88))
        );
        assertEnumMapEquals(
            wheelVelocities(44.74, 78.24, 11.25, 44.75),
            math.inverse(botVelocity(44.75, 16.75, -44.63))
        );
        assertEnumMapEquals(
            wheelVelocities(8.75, -7.74, 11.24, -5.25),
            math.inverse(botVelocity(1.75, -8.25, 3.33))
        );
    }

}
