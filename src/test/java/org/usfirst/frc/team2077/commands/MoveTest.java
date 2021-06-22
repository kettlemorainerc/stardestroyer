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
}
