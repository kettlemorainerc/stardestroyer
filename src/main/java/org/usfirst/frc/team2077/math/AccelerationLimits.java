/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.math;

import org.usfirst.frc.team2077.drivetrain.DriveChassisIF;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection;

import java.util.EnumMap;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;
import static org.usfirst.frc.team2077.math.AccelerationLimits.Type.*;

public class AccelerationLimits {

    public enum Type {
        ACCELERATION,
        DECELERATION
    }

    protected final EnumMatrix<Type, VelocityDirection> LIMITS = new EnumMatrix<>(Type.class, VelocityDirection.class);;
    protected final DriveChassisIF defaultChassis;
    public static final double G = 386.09; // Acceleration of gravity in inches/second/second.

    public AccelerationLimits(double accelerationG, double decelerationG, DriveChassisIF chassis) {
        this(accelerationG, decelerationG, chassis, new double[] {1, 1, 1});
    }

    public AccelerationLimits(double accelerationG, double decelerationG, DriveChassisIF chassis, double[] scale) {
        this.defaultChassis = chassis;

        put(NORTH, accelerationG, decelerationG, scale[NORTH.ordinal()]);
        put(EAST, accelerationG, decelerationG, scale[EAST.ordinal()]);

        EnumMap<VelocityDirection, Double> max = chassis.getMaximumVelocity();
        double inchesToDegrees = max.get(ROTATION) / max.get(NORTH);

        put(ROTATION, inchesToDegrees * accelerationG, inchesToDegrees * decelerationG, scale[ROTATION.ordinal()]);
    }

    public AccelerationLimits(double[][] doubles, DriveChassisIF chassis) {
        set(NORTH, doubles[NORTH.ordinal()]);
        set(EAST, doubles[EAST.ordinal()]);
        set(ROTATION, doubles[ROTATION.ordinal()]);

        defaultChassis = chassis;
    }

    private void put(VelocityDirection d, double acceleration, double deceleration, double scale) {
        LIMITS.set(ACCELERATION, d, G * acceleration * scale);
        LIMITS.set(DECELERATION, d, G * deceleration * scale);
    }

    public void set(VelocityDirection d, double[] limits) {
        LIMITS.set(ACCELERATION, d, limits[ACCELERATION.ordinal()]);
        LIMITS.set(DECELERATION, d, limits[DECELERATION.ordinal()]);
    }

    public double[] get(VelocityDirection d, DriveChassisIF chassis) {
        double[] limits = LIMITS.getMatrix()[d.ordinal()];
        if(d == ROTATION) {
            EnumMap<VelocityDirection, Double> max = chassis.getMaximumVelocity();
            double[] adjustedLimits = new double[2];
            int accel = ACCELERATION.ordinal();
            int decel = DECELERATION.ordinal();

            adjustedLimits[accel] = limits[accel] > 0 ? limits[accel] : LIMITS.get(ACCELERATION, NORTH) * max.get(ROTATION) / max.get(NORTH);
            adjustedLimits[decel] = limits[decel] > 0 ? limits[decel] : LIMITS.get(DECELERATION, NORTH) * max.get(ROTATION) / max.get(NORTH);

            return adjustedLimits;
        }
        return LIMITS.getMatrix()[d.ordinal()];
    }

    public double[] get(VelocityDirection d) {
        return get(d, defaultChassis);
    }

    /**
     * gets the raw, unadjusted value of the limit
     * @param d Direction
     * @param t Type
     * @return the limit for {@link VelocityDirection} {@code d} and {@link Type} {@code t}
     */
    public double get(VelocityDirection d, Type t) {
        return LIMITS.get(t, d);
    }

    /**
     * @return Acceleration/deceleration limits in inches/second/second and degrees/second/second.
     */
    public double[][] get() {
        return LIMITS.getMatrix();
    }
}
