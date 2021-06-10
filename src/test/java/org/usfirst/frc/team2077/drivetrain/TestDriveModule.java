package org.usfirst.frc.team2077.drivetrain;

public class TestDriveModule implements DriveModuleIF {
    private final double maxSpeed;
    private double currentVelocity;

    public TestDriveModule(double maxSpeed) {
        this.maxSpeed = maxSpeed;
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
}
