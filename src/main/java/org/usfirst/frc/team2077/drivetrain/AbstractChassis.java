/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.Clock;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.math.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;

public abstract class AbstractChassis extends SubsystemBase implements DriveChassisIF {

    private static <T> EnumMap<VelocityDirection, T> defaultedDirectionMap(T defaultValue) {
        EnumMap<VelocityDirection, T> newMap = new EnumMap<>(VelocityDirection.class);
        for(VelocityDirection d : VelocityDirection.values()) newMap.put(d, defaultValue);
        return newMap;
    }

    public final EnumMap<WheelPosition, DriveModuleIF> driveModule_;
    // Velocity setpoint.
    protected EnumMap<VelocityDirection, Double> setVelocity = defaultedDirectionMap(0d);
    // Velocity setpoint after adjustments for acceleration and velocity.
    protected EnumMap<VelocityDirection, Double> calculatedVelocity = defaultedDirectionMap(0d);
    protected AccelerationLimits accelerationLimits = new AccelerationLimits(0.5, 0.5, this);

    protected final Position positionSet_ = new Position(); // Continuously updated by integrating velocity setpoints.
    protected final Position positionMeasured_ = new Position(); // Continuously updated by integrating measured velocities.

    protected double maximumSpeed_;
    protected double maximumRotation_;
    protected double minimumSpeed_;
    protected double minimumRotation_;

    protected static final double G = 386.09; // Acceleration of gravity in inches/second/second.

    protected double lastUpdateTime_ = 0;
    protected double timeSinceLastUpdate_ = 0;

    protected EnumMap<VelocityDirection, Double> velocityMeasured_;

    // Debug flag gets set to true every Nth call to beginUpdate().
    protected int debugFrequency_ = 100; // N
    private long debugCounter_ = 0; // internal counter
    public boolean debug_ = false; // Use to throttle debug output.

    public AbstractChassis(DriveModuleIF[] driveModule) {
        driveModule_ = new EnumMap<>(WheelPosition.class);
        for(DriveModuleIF module : driveModule) {
            driveModule_.put(module.getWheelPosition(), module);
        }
    }

    @Override
    public void periodic() {

        debug_ = (debugCounter_++ % debugFrequency_) == 0;

        double now = Clock.getSeconds();
        timeSinceLastUpdate_ = now - lastUpdateTime_;
        lastUpdateTime_ = now;

        updatePosition();
        calculatedVelocity.put(NORTH, limit(NORTH));
        calculatedVelocity.put(EAST, limit(EAST));
        calculatedVelocity.put(ROTATION, limit(ROTATION));
        updateDriveModules();
    }

    /**
     * Perform any position calculations neccessary to account for
     * movement since last upodate.
     * Called by {@link #periodic()}.
     */
    protected abstract void updatePosition();

    /**
     * Update drive module setpoints.
     * Called by {@link #periodic()}.
     */
    protected abstract void updateDriveModules();

    @Override
    public EnumMap<VelocityDirection, Double> getMaximumVelocity() {
        EnumMap<VelocityDirection, Double> maxVel = new EnumMap<>(VelocityDirection.class);
        maxVel.put(NORTH, maximumSpeed_);
        maxVel.put(EAST, maximumSpeed_);
        maxVel.put(ROTATION, maximumRotation_);
        return maxVel;
    }

    @Override
    public EnumMap<VelocityDirection, Double> getMinimumVelocity() {
        EnumMap<VelocityDirection, Double> maxVel = new EnumMap<>(VelocityDirection.class);
        maxVel.put(NORTH, minimumSpeed_);
        maxVel.put(EAST, minimumSpeed_);
        maxVel.put(ROTATION, minimumRotation_);
        return maxVel;
    }

    @Override
    public void moveAbsolute(double north, double east, double heading) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveAbsolute(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void rotateAbsolute(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveRelative(double north, double east, double heading) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveRelative(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void rotateRelative(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPosition(double north, double east, double heading) {
        positionSet_.set(north, east, heading);
        positionMeasured_.set(north, east, heading);
    }

    @Override
    public Position getPosition() {
        return positionSet_.copy();
    }

    @Override
    public void setVelocity(double north, double east, double clockwise) {
        setVelocity(north, east, clockwise, accelerationLimits);
    }
    @Override
    public void setVelocity(double north, double east) {
        setVelocity(north, east, accelerationLimits);
    }
    @Override
    public void setRotation(double clockwise) {
        setRotation(clockwise, accelerationLimits);
    }

    @Override
    public final void setVelocity01(double north, double east, double clockwise) {
        EnumMap<VelocityDirection, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(NORTH), east * max.get(EAST), clockwise * max.get(ROTATION));
    }

    @Override
    public final void setVelocity01(double north, double east) {
        EnumMap<VelocityDirection, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(NORTH), east * max.get(EAST));
    }

    @Override
    public final void setRotation01(double clockwise) {
        setRotation(clockwise * getMaximumVelocity().get(ROTATION));
    }

    @Override
    public void halt() {
        double max = 10000;
        setVelocity(0, 0, 0, new AccelerationLimits(new double[][] {{max, max}, {max, max}, {max, max}}, this));
    }

    @Override
    public EnumMap<VelocityDirection, Double> getVelocityMeasured() {
        return getVelocityCalculated();
    }

    @Override
    public void setGLimits(double accelerationG, double decelerationG) {
        accelerationLimits.set(NORTH, accelerationG * G, decelerationG * G);
        double eastAccelerationRatio = .7;
        accelerationLimits.set(EAST, accelerationG * G * eastAccelerationRatio, decelerationG * G * eastAccelerationRatio);
        accelerationLimits.set(ROTATION, 0, 0);
    }

    @Override
    public double[][] getAccelerationLimits() {
        return accelerationLimits.get();
    }

    protected double limit(VelocityDirection direction) {
        double newV = calculatedVelocity.get(direction), oldV = setVelocity.get(direction);
        return limit(newV, oldV, maximumSpeed_, accelerationLimits.get(direction));
    }

    /**
     * Limit set point change for linear or rotational set points based on velocity or acceleration constraints.
     * Depends on {@link #timeSinceLastUpdate_} being called at the beginning of {@link #setVelocity}
     * and {@link #setRotation}.
//     * @param newV
//     * @param oldV
//     * @param maxV
//     * @param accelerationLimits
     * @return Acceleration/range constrained set point.
     */
    protected double limit(double newV, double oldV, double maxV, double[] accelerationLimits) {
        boolean accelerating = Math.abs(newV) >= Math.abs(oldV) && Math.signum(newV) == Math.signum(oldV);
        double deltaLimit = (accelerating ? accelerationLimits[0] : accelerationLimits[1]) * timeSinceLastUpdate_; // always positive
        double deltaRequested = newV - oldV;
        double delta = Math.min(deltaLimit, Math.abs(deltaRequested)) * Math.signum(deltaRequested);
        double v = oldV + delta;
        return Math.max(-maxV, Math.min(maxV, v));
    }

    /**
     * Cartesian to polar coordinate conversion.
     * @param north (inches)
     * @param east (inches)
     * @return {
     * <br> magnitude (inches)
     * <br> direction (degrees clockwise from north)
     * <br>}
     */
    public static double[] cartesianToPolar(double north, double east) {
        double magnitude = Math.sqrt(north*north + east*east);
        double direction = Math.toDegrees(Math.atan2(north, east));
        return new double[] {magnitude, direction};
    }
    /**
     * Polar to cartesian coordinate conversion.
     * @param magnitude (inches)
     * @param direction (degrees clockwise from north)
     * @return {
     * <br> north (inches)
     * <br> east (inches)
     * <br>}
     */
    public static double[] polarToCartesian(double magnitude, double direction) {
        double north = magnitude * Math.cos(direction);
        double east = magnitude * Math.sin(direction);
        return new double[] {north, east};
    }
}
