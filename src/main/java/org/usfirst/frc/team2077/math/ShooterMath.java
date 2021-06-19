package org.usfirst.frc.team2077.math;

public class ShooterMath {

    private double height, shooterAngle, distance, shooterRPM, neededAngle;
    private final double barrelLength = 11.375, shooterHeightFromActuator = 3.75; //inches
    private final double actuatorLength = 13.75, angleToPerpendicular = 3, angleOffset = -.7;
    private double lastActuatorTicks;
//    private final int maxAngle = 58, minAngle = 38;

    public double[] getRangeAV() {
        return new double[] {shooterAngle, shooterRPM};
    }

    public double[] getRangeAV(double distance, double actuatorTicks) {
        this.setDistance(distance, actuatorTicks);
        return new double[] {shooterAngle, shooterRPM};
    }

    public void setDistance(double distance, double actuatorTicks) {
        this.distance = distance;
        if(actuatorTicks != this.lastActuatorTicks) updateAngle(actuatorTicks);
        lastActuatorTicks = actuatorTicks;
        this.updateRPM();
        this.updateNeededAngle();
    }

    private double getAngle() {
        return this.shooterAngle;
    }

    private double getRPM() { return this.shooterRPM; }

    public   double getNeededAngle(boolean actuatorTicks) { return actuatorTicks ? degreesToTicks(this.neededAngle)  : this.neededAngle; }

    private double updateAngle(double actuatorTickDistance) {
        double actuatedDistance = actuatorTickDistance / 6.38e6;
        double numerator = (-Math.pow(actuatorLength + actuatedDistance,2)) + Math.pow(barrelLength,2) + Math.pow(shooterHeightFromActuator,2);
        double denominator = 2 * barrelLength * shooterHeightFromActuator;
        this.shooterAngle = Math.toDegrees(Math.acos((numerator/denominator))) - 90 + angleToPerpendicular + angleOffset;
        System.out.println("ShooterAngle " + this.shooterAngle);

        return this.shooterAngle;
    }

    private double degreesToTicks(double degrees) {
        double denominator = 2 * barrelLength * shooterHeightFromActuator;

        double x = Math.cos(Math.toRadians(degrees - angleToPerpendicular + 90));

        return 6.38e6 * (Math.sqrt(-(denominator * x
                - Math.pow(barrelLength,2) - Math.pow(shooterHeightFromActuator,2)))
                - actuatorLength);
    }

    private void updateNeededAngle() {
        if (distance > 108) {
            this.neededAngle = 35.3;
        } else {
            this.neededAngle = 60.32 * Math.exp(-0.005 * distance);
        }
    }

    private void updateRPM() {
        if (distance > 108) { //long shot
            this.shooterRPM = 2243.5 * Math.exp(0.0014*distance);
        } else {
            //short shot
            this.shooterRPM = 5600;
        }
    }

}
