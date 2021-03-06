/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.math;

import org.usfirst.frc.team2077.drivetrain.DriveChassisIF;

public class Acceleration {

    protected final double[] north_;
    protected final double[] east_;
    protected final double[] rotation_;

    public static final double G = 386.09; // Acceleration of gravity in inches/second/second.

    public Acceleration(double accelerationG, double decelerationG, DriveChassisIF chassis) {
        this(accelerationG, decelerationG, chassis, new double[] {1, 1, 1});
    }

    public Acceleration(double accelerationG, double decelerationG, DriveChassisIF chassis, double[] scale) {
        north_ = new double[] {G * accelerationG * scale[0], G * decelerationG * scale[0]};
        east_ = new double[] {G * accelerationG * scale[1], G * decelerationG * scale[1]};
        double[] max = chassis.getMaximumVelocity();
        double inchesToDegrees = max[2] / max[0];
        rotation_ = new double[] {inchesToDegrees * G * accelerationG * scale[2], inchesToDegrees * G * decelerationG * scale[2]};
    }

    /**
     * @return Acceleration/deceleration limits in inches/second/second and degrees/second/second.
     */
    public double[][] get() {
        return new double[][] {north_, east_, rotation_};
    }
}
