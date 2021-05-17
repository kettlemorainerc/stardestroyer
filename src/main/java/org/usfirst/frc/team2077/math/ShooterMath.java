package org.usfirst.frc.team2077.math;

public class ShooterMath {

    private double height, shooterAngle, distance, shooterRPM;
    private final double barrelLength = 11.375, shooterHeightFromActuator = 3.75; //inches
    private final double actuatorLength = 13.75, AngleToPerpendicular = 6;
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
        this.updateAngle(actuatorTicks);
        this.updateRPM();
    }

    private double getAngle() {
        return this.shooterAngle;
    }

    private double getRPM() { return this.shooterRPM; }

    private double updateAngle(double actuatorTicks) {
        double actuatedDistance = actuatorTicks / (8.65e5) / 10;
        double numerator = -Math.pow(actuatorLength + actuatedDistance,2) + Math.pow(barrelLength,2) + Math.pow(shooterHeightFromActuator,2);
        double denominator = 2 * barrelLength * shooterHeightFromActuator;
        this.shooterAngle = Math.toDegrees(Math.acos((numerator/denominator))) - 90 + AngleToPerpendicular;

        return this.shooterAngle;
    }

    private void updateRPM() {
        //all distances in meters as of writing this
        double height = 2.5 - (.26 * Math.tan(shooterAngle));
        double initialVelocity = Math.sqrt(
                4.9 * Math.pow(distance, 2) /
                (Math.pow(Math.cos(shooterAngle), 2) * (Math.tan(shooterAngle) * distance - height))
        );
        //Conversion constant from distance/second to rotations/second on the shooter wheels
        double rpm = 187.98 * initialVelocity;

        this.shooterRPM = rpm == 0 ? shooterRPM : rpm;
    }

}
