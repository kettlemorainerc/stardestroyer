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

    // this tends to get called every period in chassis
    @Override
    public void setVelocity(double velocity) {
        currentVelocity = velocity;
//        System.out.println("New velocity: " + currentVelocity + "target: " + velocity);
    }

    /**
     * Should ONLY get called by robot code. NOT test code
     * @return
     */
    @Override
    public double getVelocity() {
        return currentVelocity;
    }

    public double safeGetVelocity() {
        return currentVelocity;
    }

    // I don't think the following are EVER used
    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public void resetDistance() {

    }
}
