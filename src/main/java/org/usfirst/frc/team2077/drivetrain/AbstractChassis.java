/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.Clock;
import org.usfirst.frc.team2077.drivetrain.MecanumMath.*;
import org.usfirst.frc.team2077.math.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team2077.math.AccelerationLimits.*;

import java.util.EnumMap;
import java.util.function.*;

import static org.usfirst.frc.team2077.drivetrain.MecanumMath.VelocityDirection.*;

public abstract class AbstractChassis extends SubsystemBase implements DriveChassisIF {

    private static <T> EnumMap<VelocityDirection, T> defaultedDirectionMap(T defaultValue) {
        EnumMap<VelocityDirection, T> newMap = new EnumMap<>(VelocityDirection.class);
        for(VelocityDirection d : VelocityDirection.values()) newMap.put(d, defaultValue);
        return newMap;
    }

    public final EnumMap<WheelPosition, DriveModuleIF> driveModule_;
    protected final double wheelbase_;
    protected final double trackWidth_;
    protected final double wheelRadius_;
    protected final Supplier<Double> getSeconds;

    protected double maximumSpeed_;
    protected double maximumRotation_;
    protected double minimumSpeed_;
    protected double minimumRotation_;

//    public static final double G = 386.09; // Acceleration of gravity in inches/second/second.
    // Ideally accel/decel values are set just below wheelspin or skidding to a stop.
    // Optimal values are highly dependent on wheel/surface traction and somewhat on
    // weight distribution.
    // For safety err on the low side for acceleration, high for deceleration.
    protected AccelerationLimits accelerationLimits = new AccelerationLimits(false, .5, .5, this);
//    protected double[] northAccelerationLimit_ = {G/2, G/2}; // maximum acceleration/deceleration in inches/second/second.
//    protected double[] eastAccelerationLimit_ = {G/2, G/2}; // maximum acceleration/deceleration in inches/second/second.
//    protected double[] rotationAccelerationLimit_ = {0, 0}; // maximum acceleration/deceleration in tangential inches/second/second.

    protected double lastUpdateTime_ = 0;
    protected double timeSinceLastUpdate_ = 0;

    protected final Position positionSet_ = new Position(); // Continuously updated by integrating velocity setpoints.
    protected final Position positionMeasured_ = new Position(); // Continuously updated by integrating measured velocities.

    protected EnumMap<VelocityDirection, Double> velocity = defaultedDirectionMap(0d); // target velocity for next period
    protected EnumMap<VelocityDirection, Double> targetVelocity = defaultedDirectionMap(0d);
    protected EnumMap<VelocityDirection, Double> velocitySet_ = defaultedDirectionMap(0d); // target velocity overall
    protected EnumMap<VelocityDirection, Double> velocityMeasured_ = defaultedDirectionMap(0d); //

    // Debug flag gets set to true every Nth call to beginUpdate().
    protected int debugFrequency_ = 100; // N
    private long debugCounter_ = 0; // internal counter
    public boolean debug_ = false; // Use to throttle debug output.

    public AbstractChassis(EnumMap<WheelPosition, DriveModuleIF> driveModule, double wheelbase, double trackWidth, double wheelRadius, Supplier<Double> getSeconds) {
        driveModule_ = driveModule;
        wheelbase_ = wheelbase;
        trackWidth_ = trackWidth;
        wheelRadius_ = wheelRadius;
        this.getSeconds = getSeconds;
    }

    public AbstractChassis(EnumMap<WheelPosition, DriveModuleIF> driveModule_, double wheelbase, double trackWidth, double wheelRadius) {
        this(driveModule_, wheelbase, trackWidth, wheelRadius, Clock::getSeconds);
    }

    @Override
    public void periodic() {

        debug_ = (debugCounter_++ % debugFrequency_) == 0;

        double now = getSeconds.get();
        timeSinceLastUpdate_ = now - lastUpdateTime_;
        lastUpdateTime_ = now;

        updatePosition();
        limitVelocity(NORTH, maximumSpeed_);
        limitVelocity(EAST, maximumSpeed_);
        limitVelocity(ROTATION, maximumRotation_);
//        velocity.compute(NORTH, (k, north) -> limit(targetVelocity.get(NORTH), north, maximumSpeed_, accelerationLimits.get(NORTH)));
//        velocity.compute(EAST, (k, east) -> limit(targetVelocity.get(EAST), east, maximumSpeed_, accelerationLimits.get(NORTH)));
//        velocity.compute(ROTATION, (k, rotation) -> limit(targetVelocity.get(ROTATION), rotation, maximumRotation_, accelerationLimits.get(ROTATION)));
//        north_ = limit(northSet_, north_, maximumSpeed_, northAccelerationLimit_);
//        east_ = limit(eastSet_, east_, maximumSpeed_, eastAccelerationLimit_);
//        clockwise_ = limit(clockwiseSet_, clockwise_, maximumRotation_, rotationAccelerationLimit_);
        updateDriveModules();
    }

    protected void limitVelocity(VelocityDirection direction, double max) {
        double currentVelocity = this.velocity.get(direction);
        double targetVelocity = this.targetVelocity.get(direction);

        boolean accelerating = Math.abs(targetVelocity) >= Math.abs(currentVelocity) && Math.signum(targetVelocity) == Math.signum(currentVelocity);
        double deltaLimit = accelerationLimits.get(direction, accelerating ? Type.ACCELERATION : Type.DECELERATION) * timeSinceLastUpdate_; // always positive
        double deltaRequested = targetVelocity - currentVelocity;
        double delta = Math.min(deltaLimit, Math.abs(deltaRequested)) * Math.signum(deltaRequested);
        double v = currentVelocity + delta;
        double newDirectionVelocity = Math.max(-max, Math.min(max, v));

        velocity.put(direction, newDirectionVelocity);
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
    public EnumMap<VelocityDirection, Double> getVelocitySet() {
        return targetVelocity.clone();
    }

    @Override
    public EnumMap<VelocityDirection, Double> getVelocityCalculated() {
        return velocity.clone();
    }

    @Override
    public EnumMap<VelocityDirection, Double> getVelocityMeasured() {
        return velocityMeasured_.clone();
    }

    @Override
    public EnumMap<VelocityDirection, Double> getMaximumVelocity() {
        EnumMap<VelocityDirection, Double> stuff = new EnumMap<>(VelocityDirection.class);

        stuff.put(NORTH, maximumSpeed_);
        stuff.put(EAST, maximumSpeed_);
        stuff.put(ROTATION, maximumRotation_);

        return stuff;
//        return new double[] {maximumSpeed_, maximumSpeed_, maximumRotation_};
    }

    @Override
    public EnumMap<VelocityDirection, Double> getMinimumVelocity() {
        EnumMap<VelocityDirection, Double> stuff = new EnumMap<>(VelocityDirection.class);

        stuff.put(NORTH, minimumSpeed_);
        stuff.put(EAST, minimumSpeed_);
        stuff.put(ROTATION, minimumRotation_);

        return stuff;
//        return new double[] {minimumSpeed_, minimumSpeed_, minimumRotation_};
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
        return positionSet_;
//        return positionSet_.get();
        //return positionMeasured_.get();
    }

    @Override
    public void setVelocity(double north, double east, double clockwise) {
        setVelocity(north, east, clockwise, getAccelerationLimits());
    }
    @Override
    public void setVelocity(double north, double east) {
        setVelocity(north, east, getAccelerationLimits());
    }
    @Override
    public void setRotation(double clockwise) {
        setRotation(clockwise, getAccelerationLimits());
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
    public void setGLimits(double accelerationG, double decelerationG) {
//        accelerationG_ = accelerationG;
//        decelerationG_ = decelerationG;
//        northAccelerationLimit_ = new double[] {accelerationG * G, decelerationG * G};
        double eastAccelerationRatio = .7;
        accelerationLimits = new AccelerationLimits(false, accelerationG, decelerationG, this, new double[]{1, eastAccelerationRatio, 1});
//        eastAccelerationLimit_ = new double [] {northAccelerationLimit_[0] * eastAccelerationRatio, northAccelerationLimit_[1] * eastAccelerationRatio};
//        rotationAccelerationLimit_ = new double[] {0, 0};
    }

    @Override
    public AccelerationLimits getAccelerationLimits() {
//        double[] max = getMaximumVelocity();
//        double[] rotationAccelerationLimit = {
//            rotationAccelerationLimit_[0]>0 ? rotationAccelerationLimit_[0] : (northAccelerationLimit_[0]*max[2]/max[0]),
//            rotationAccelerationLimit_[1]>0 ? rotationAccelerationLimit_[1] : (northAccelerationLimit_[1]*max[2]/max[0])};
//        return new double[][] {northAccelerationLimit_, eastAccelerationLimit_, rotationAccelerationLimit};

        return accelerationLimits.getAdjustedAdjustments();
    }
}
