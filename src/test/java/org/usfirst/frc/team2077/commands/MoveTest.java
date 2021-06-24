package org.usfirst.frc.team2077.commands;

import org.junit.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

import java.util.*;

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
        RobotTest.includeCommands(new Move(8, 0, 0));
        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(1.6, 0.0, 0.0)
                               .measuredPosition(1.6, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(3.2, 0.0, 0.0)
                               .measuredPosition(3.2, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(4.8, 0.0, 0.0)
                               .measuredPosition(4.8, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(6.4, 0.0, 0.0)
                               .measuredPosition(6.4, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(0.8, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(8, 0.0, 0.0)
                               .measuredPosition(8, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(.8, .8, .8, .8)
                               .calculatedVelocities(.8, 0.0, 0.0)
                               .setVelocities(8.0, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(9.6, 0.0, 0.0)
                               .measuredPosition(9.6, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(8.0, 8.0, 8.0, 8.0)
                               .calculatedVelocities(8.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(.8, 0.0, 0.0)
                               .setPosition(9.76, 0.0, 0.0)
                               .measuredPosition(9.76, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(8.0, 0.0, 0.0)
                               .setPosition(11.36, 0.0, 0.0)
                               .measuredPosition(11.36, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(11.36, 0.0, 0.0)
                               .measuredPosition(11.36, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(11.36, 0.0, 0.0)
                               .measuredPosition(11.36, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(11.36, 0.0, 0.0)
                               .measuredPosition(11.36, 0.0, 0.0)
        );
    }

    @Test
    public void moves_8_east_updates_chassis_correctly() {
        RobotTest.includeCommands(new Move(0, 8, 0));

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 1.04, 0.0)
                               .measuredPosition(0, 1.04, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 2.08, 0.0)
                               .measuredPosition(0, 2.08, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 3.12, 0.0)
                               .measuredPosition(0, 3.12, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 4.16, 0.0)
                               .measuredPosition(0, 4.16, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 0.8, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 5.2, 0.0)
                               .measuredPosition(0, 5.2, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-.8, .8, -.8, .8)
                               .calculatedVelocities(0.0, .8, 0.0)
                               .setVelocities(0.0, 8.0, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 6.24, 0.0)
                               .measuredPosition(0, 6.24, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, 8.0, -8.0, 8.0)
                               .calculatedVelocities(0.0, 8.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.52, 0.0)
                               .setPosition(0, 6.344, 0.0)
                               .measuredPosition(0, 6.344, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 5.2, 0.0)
                               .setPosition(0, 7.384, 0.0)
                               .measuredPosition(0, 7.384, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0, 7.384, 0.0)
                               .measuredPosition(0, 7.384, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0, 7.384, 0.0)
                               .measuredPosition(0, 7.384, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0, 7.384, 0.0)
                               .measuredPosition(0, 7.384, 0.0)
        );
    }

    @Test
    public void rotate_8_degrees_correctly_updates_chassis() {
        RobotTest.includeCommands(new Move(8));

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 21.319359818821333)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, -8.0, 8.0, 8.0)
                               .calculatedVelocities(0.0, 0.0, 21.319359818821333)
                               .setVelocities(0.0, 0.0, 21.319359818821333)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, -8.0, 8.0, 8.0)
                               .calculatedVelocities(0.0, 0.0, 21.319359818821333)
                               .setVelocities(0.0, 0.0, 21.319359818821333)
                               .measuredVelocities(0.0, 0.0, 21.319359818821333)
                               .setPosition(0.0, 0.0, 4.263871963764268)
                               .measuredPosition(0.0, 0.0, 4.263871963764268)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-8.0, -8.0, 8.0, 8.0)
                               .calculatedVelocities(0.0, 0.0, 21.319359818821333)
                               .setVelocities(0.0, 0.0, 2.1319359818821333)
                               .measuredVelocities(0.0, 0.0, 21.319359818821333)
                               .setPosition(0.0, 0.0, 8.527743927528533)
                               .measuredPosition(0.0, 0.0, 8.527743927528533)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-0.8, -0.8, 0.8, 0.8)
                               .calculatedVelocities(0.0, 0.0, 2.1319359818821333)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 21.319359818821333)
                               .setPosition(0.0, 0.0, 12.7916158912928)
                               .measuredPosition(0.0, 0.0, 12.7916158912928)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 2.1319359818821337)
                               .setPosition(0.0, 0.0, 13.218003087669226)
                               .measuredPosition(0.0, 0.0, 13.218003087669226)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 13.218003087669226)
                               .measuredPosition(0.0, 0.0, 13.218003087669226)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 13.218003087669226)
                               .measuredPosition(0.0, 0.0, 13.218003087669226)
        );
    }

    @Test
    public void move_8_north_east_updates_chassis_correctly() {
        RobotTest.includeCommands(new Move(8, 8));

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(0.0, 0.0, 0.0)
                               .measuredPosition(0.0, 0.0, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(1.6, 0.7072000000000004, 0.0)
                               .measuredPosition(0.9523809523809527, 0.42095238095238113, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(3.2, 1.4144, 0.0)
                               .measuredPosition(1.9047619047619047, 0.8419047619047619, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(4.8, 2.1216, 0.0)
                               .measuredPosition(2.8571428571428568, 1.2628571428571427, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(6.3999999999999995, 2.8287999999999998, 0.0)
                               .measuredPosition(3.809523809523809, 1.6838095238095234, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(7.999999999999999, 3.5359999999999996, 0.0)
                               .measuredPosition(4.761904761904761, 2.1047619047619044, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(8.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(9.599999999999998, 4.2432, 0.0)
                               .measuredPosition(5.7142857142857135, 2.5257142857142854, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(1.5238095238095235, 8.0, 1.5238095238095235, 8.0)
                               .calculatedVelocities(8.0, 5.44, 0.0)
                               .setVelocities(0.0, 5.44, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(11.199999999999998, 4.9504, 0.0)
                               .measuredPosition(6.666666666666666, 2.9466666666666663, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-5.44, 5.44, -5.44, 5.44)
                               .calculatedVelocities(0.0, 5.44, 0.0)
                               .setVelocities(0.0, 0.8, 0.0)
                               .measuredVelocities(4.761904761904762, 2.104761904761905, 0.0)
                               .setPosition(12.799999999999997, 5.6576, 0.0)
                               .measuredPosition(7.619047619047619, 3.3676190476190473, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(-0.7999999999999998, 0.7999999999999998, -0.7999999999999998, 0.7999999999999998)
                               .calculatedVelocities(0.0, 0.7999999999999998, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 3.5360000000000005, 0.0)
                               .setPosition(12.799999999999997, 6.364800000000001, 0.0)
                               .measuredPosition(7.619047619047619, 4.074819047619047, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.5199999999999999, 0.0)
                               .setPosition(12.799999999999997, 6.468800000000001, 0.0)
                               .measuredPosition(7.619047619047619, 4.178819047619047, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(12.799999999999997, 6.468800000000001, 0.0)
                               .measuredPosition(7.619047619047619, 4.178819047619047, 0.0)
        );

        assertPeriodicUpdate(
            new ChassisValues().wheelVelocities(0.0, 0.0, 0.0, 0.0)
                               .calculatedVelocities(0.0, 0.0, 0.0)
                               .setVelocities(0.0, 0.0, 0.0)
                               .measuredVelocities(0.0, 0.0, 0.0)
                               .setPosition(12.799999999999997, 6.468800000000001, 0.0)
                               .measuredPosition(7.619047619047619, 4.178819047619047, 0.0)
        );
    }
}
