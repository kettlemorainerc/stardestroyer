package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;

public class TestDriveModule implements DriveModuleIF {
    private final double maxSpeed;
    private double currentVelocity;
    private final WheelPosition position;

    public TestDriveModule(double maxSpeed, WheelPosition position) {
        this.maxSpeed = maxSpeed;
        this.position = position;
    }

    @Override
    public double getMaximumSpeed() {
        return maxSpeed;
    }

    @Override
    public void setVelocity(double velocity) {
        currentVelocity = velocity;
    }

    @Override
    public WheelPosition getWheelPosition() {
        return position;
    }

    @Override
    public double getVelocity() {
        return currentVelocity;
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public void resetDistance() {

    }

    public String toString() {
        return String.format("[Test %s Vel: %s / %s]", position, currentVelocity, maxSpeed);
    }
}
