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

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.Direction.*;

public abstract class AbstractChassis extends SubsystemBase implements DriveChassisIF {

    private static <T> EnumMap<Direction, T> defaultedDirectionMap(T defaultValue) {
        EnumMap<Direction, T> newMap = new EnumMap<>(Direction.class);
        for(Direction d : Direction.values()) newMap.put(d,defaultValue);
        return newMap;
    }

    public final EnumMap<WheelPosition, DriveModuleIF> driveModule_;
    protected EnumMap<Direction, Double> setVelocity = defaultedDirectionMap(0d);
    protected EnumMap<Direction, Double> calculatedVelocity = defaultedDirectionMap(0d);
    protected AccelerationLimits accelerationLimits;

    // Velocity setpoint.
    protected double northSet_ = 0;
    protected double eastSet_ = 0;
    protected double clockwiseSet_ = 0;

    // Velocity setpoint after adjustments for acceleration and velocity.
    protected double north_ = 0;
    protected double east_ = 0;
    protected double clockwise_ = 0;

    protected double maximumSpeed_;
    protected double maximumRotation_;
    protected double minimumSpeed_;
    protected double minimumRotation_;

    protected static final double G = 386.09; // Acceleration of gravity in inches/second/second.
    // Ideally accel/decel values are set just below wheelspin or skidding to a stop.
    // Optimal values are highly dependent on wheel/surface traction and somewhat on
    // weight distribution.
    // For safety err on the low side for acceleration, high for deceleration.
    protected double[] northAccelerationLimit_ = {G/2, G/2}; // maximum acceleration/deceleration in inches/second/second.
    protected double[] eastAccelerationLimit_ = {G/2, G/2}; // maximum acceleration/deceleration in inches/second/second.
    protected double[] rotationAccelerationLimit_ = {0, 0}; // maximum acceleration/deceleration in tangential inches/second/second.

    protected double lastUpdateTime_ = 0;
    protected double timeSinceLastUpdate_ = 0;

    protected final Position positionSet_ = new Position(); // Continuously updated by integrating velocity setpoints.
    protected final Position positionMeasured_ = new Position(); // Continuously updated by integrating measured velocities.

//    protected double[] velocitySet_ = {0, 0, 0};
    protected EnumMap<Direction, Double> velocityMeasured_;

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
        calculatedVelocity.put(CLOCKWISE, limit(CLOCKWISE));
//        north_ = limit(northSet_, north_, maximumSpeed_, northAccelerationLimit_);
//        east_ = limit(eastSet_, east_, maximumSpeed_, eastAccelerationLimit_);
//        clockwise_ = limit(clockwiseSet_, clockwise_, maximumRotation_, rotationAccelerationLimit_);
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
    public EnumMap<Direction, Double> getMaximumVelocity() {
        EnumMap<Direction, Double> maxVel = new EnumMap<>(Direction.class);
        maxVel.put(NORTH, maximumSpeed_);
        maxVel.put(EAST, maximumSpeed_);
        maxVel.put(CLOCKWISE, maximumRotation_);
        return maxVel;
    }

    @Override
    public EnumMap<Direction, Double> getMinimumVelocity() {
        EnumMap<Direction, Double> maxVel = new EnumMap<>(Direction.class);
        maxVel.put(NORTH, minimumSpeed_);
        maxVel.put(EAST, minimumSpeed_);
        maxVel.put(CLOCKWISE, minimumRotation_);
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
        //return positionMeasured_.get();
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
        EnumMap<Direction, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(NORTH), east * max.get(EAST), clockwise * max.get(CLOCKWISE));
    }

    @Override
    public final void setVelocity01(double north, double east) {
        EnumMap<Direction, Double> max = getMaximumVelocity();
        setVelocity(north * max.get(NORTH), east * max.get(EAST));
    }

    @Override
    public final void setRotation01(double clockwise) {
        setRotation(clockwise * getMaximumVelocity().get(CLOCKWISE));
    }

    @Override
    public void halt() {
        double max = 10000;
        setVelocity(0, 0, 0, new AccelerationLimits(new double[][] {{max, max}, {max, max}, {max, max}}, this));
    }

    @Override
    public EnumMap<Direction, Double> getVelocityMeasured() {
        return getVelocityCalculated();
    }

    @Override
    public void setGLimits(double accelerationG, double decelerationG) {
        //accelerationG_ = accelerationG;
        //decelerationG_ = decelerationG;
        northAccelerationLimit_ = new double[] {accelerationG * G, decelerationG * G};
        double eastAccelerationRatio = .7;
        eastAccelerationLimit_ = new double [] {northAccelerationLimit_[0] * eastAccelerationRatio, northAccelerationLimit_[1] * eastAccelerationRatio};
        rotationAccelerationLimit_ = new double[] {0, 0};
    }

    @Override
    public double[][] getAccelerationLimits() {
        EnumMap<Direction, Double> max = getMaximumVelocity();
        double[] rotationAccelerationLimit = {
            rotationAccelerationLimit_[0]>0 ?
                rotationAccelerationLimit_[0] :
                (northAccelerationLimit_[0]*max.get(CLOCKWISE)/max.get(NORTH)),
            rotationAccelerationLimit_[1]>0 ?
                rotationAccelerationLimit_[1] :
                (northAccelerationLimit_[1]*max.get(CLOCKWISE)/max.get(NORTH))};
        return new double[][] {northAccelerationLimit_, eastAccelerationLimit_, rotationAccelerationLimit};
    }

    protected double limit(Direction direction) {
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
        
        //return Math.max(-maxV, Math.min(maxV, newV));
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
