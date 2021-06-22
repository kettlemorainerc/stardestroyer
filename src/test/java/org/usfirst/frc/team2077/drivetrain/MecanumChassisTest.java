package org.usfirst.frc.team2077.drivetrain;

import org.junit.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.math.Position;

import java.util.*;

import static org.junit.Assert.*;
import static org.usfirst.frc.team2077.math.AccelerationLimits.*;

public class MecanumChassisTest extends ChassisTest<MecanumChassis> {
    @Override
    protected MecanumChassis create(EnumMap<WheelPosition, DriveModuleIF> driveModule) {
        return new MecanumChassis(driveModule, TestClock::getAndIncrementSeconds);
    }

    @Override
    public void beforeEachTest() {
        RobotTest.forTeleop();
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
}
